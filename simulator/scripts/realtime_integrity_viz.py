#!/usr/bin/env python3
"""Bounded, time-aligned RViz bridge for realtime UWB/IMU integrity."""

import copy
import math
import threading
from collections import OrderedDict, deque

import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from uwb_imu_pl.msg import IntegrityStatus
from visualization_msgs.msg import Marker, MarkerArray


class RealtimeIntegrityViz:
    """Publish one fused/PL/ground-truth comparison at each integrity epoch."""

    ELLIPSOID_ID = 0
    HPL_RING_ID = 1
    VPL_LINE_ID = 2
    STATUS_TEXT_ID = 3

    def __init__(self):
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.sample_period_ns = int(
            max(0.001, float(rospy.get_param("~path_sample_period_s", 0.05)))
            * 1.0e9
        )
        self.path_publish_period_s = max(
            0.05, float(rospy.get_param("~path_publish_period_s", 0.2))
        )
        self.path_history_ns = int(
            max(1.0, float(rospy.get_param("~path_history_s", 120.0))) * 1.0e9
        )
        history_limit = int(self.path_history_ns // self.sample_period_ns) + 2
        self.max_path_points = max(
            1,
            min(
                int(rospy.get_param("~max_path_points", history_limit)),
                history_limit,
            ),
        )
        self.gt_buffer_history_ns = int(
            max(
                1.0,
                float(rospy.get_param("~ground_truth_buffer_s", 120.0)),
            )
            * 1.0e9
        )
        self.max_gt_buffer = max(
            2, int(rospy.get_param("~max_ground_truth_messages", 30000))
        )
        self.max_pending = max(
            2, int(rospy.get_param("~max_pending_messages", 2000))
        )
        self.max_drawable_pl_m = max(
            0.01, float(rospy.get_param("~max_drawable_pl_m", 50.0))
        )
        self.ring_segments = max(24, int(rospy.get_param("~ring_segments", 96)))

        gt_topic = rospy.get_param("~ground_truth_topic", "/sim/odom")
        fused_topic = rospy.get_param(
            "~fused_odometry_topic", "/uwb_imu_pl/odometry"
        )
        integrity_topic = rospy.get_param(
            "~integrity_topic", "/uwb_imu_pl/integrity"
        )
        aligned_gt_topic = rospy.get_param(
            "~time_aligned_ground_truth_topic",
            "/uwb_imu_pl/viz/time_aligned_ground_truth",
        )
        fused_epoch_topic = rospy.get_param(
            "~fused_epoch_topic", "/uwb_imu_pl/viz/fused_epoch"
        )
        aligned_gt_path_topic = rospy.get_param(
            "~time_aligned_ground_truth_path_topic",
            rospy.get_param(
                "~ground_truth_path_topic",
                "/uwb_imu_pl/viz/time_aligned_ground_truth_path",
            ),
        )
        fused_path_topic = rospy.get_param(
            "~fused_path_topic", "/uwb_imu_pl/viz/fused_path"
        )
        marker_topic = rospy.get_param(
            "~pl_marker_topic", "/uwb_imu_pl/viz/pl_markers"
        )

        self._lock = threading.Lock()
        self._ground_truth = deque()
        self._aligned_gt_poses = deque(maxlen=self.max_path_points)
        self._fused_poses = deque(maxlen=self.max_path_points)
        self._last_path_sample_ns = None
        self._latest_path_stamp = rospy.Time()
        self._paths_dirty = False
        self._pending_odometry = OrderedDict()
        self._pending_integrity = OrderedDict()
        self._pending_epochs = OrderedDict()

        self._aligned_gt_pub = rospy.Publisher(
            aligned_gt_topic, Odometry, queue_size=1
        )
        self._fused_epoch_pub = rospy.Publisher(
            fused_epoch_topic, Odometry, queue_size=1
        )
        self._gt_path_pub = rospy.Publisher(
            aligned_gt_path_topic, Path, queue_size=1, latch=True
        )
        self._fused_path_pub = rospy.Publisher(
            fused_path_topic, Path, queue_size=1, latch=True
        )
        self._marker_pub = rospy.Publisher(
            marker_topic, MarkerArray, queue_size=1, latch=True
        )

        self._gt_sub = rospy.Subscriber(
            gt_topic, Odometry, self._ground_truth_callback, queue_size=1000
        )
        self._fused_sub = rospy.Subscriber(
            fused_topic, Odometry, self._fused_callback, queue_size=200
        )
        self._integrity_sub = rospy.Subscriber(
            integrity_topic, IntegrityStatus, self._integrity_callback, queue_size=200
        )
        self._path_timer = rospy.Timer(
            rospy.Duration(self.path_publish_period_s), self._path_timer_callback
        )

    @staticmethod
    def _stamp_ns(stamp):
        return stamp.to_nsec()

    def _pose_stamped(self, message):
        pose = PoseStamped()
        pose.header.stamp = message.header.stamp
        pose.header.frame_id = self.world_frame
        pose.pose = copy.deepcopy(message.pose.pose)
        return pose

    @staticmethod
    def _slerp(left, right, ratio):
        q0 = [left.x, left.y, left.z, left.w]
        q1 = [right.x, right.y, right.z, right.w]
        dot = sum(a * b for a, b in zip(q0, q1))
        if dot < 0.0:
            q1 = [-value for value in q1]
            dot = -dot
        dot = max(-1.0, min(1.0, dot))
        if dot > 0.9995:
            result = [a + ratio * (b - a) for a, b in zip(q0, q1)]
        else:
            theta = math.acos(dot)
            sine = math.sin(theta)
            left_weight = math.sin((1.0 - ratio) * theta) / sine
            right_weight = math.sin(ratio * theta) / sine
            result = [
                left_weight * a + right_weight * b for a, b in zip(q0, q1)
            ]
        norm = math.sqrt(sum(value * value for value in result))
        if norm <= 1.0e-12:
            return (0.0, 0.0, 0.0, 1.0)
        return tuple(value / norm for value in result)

    def _interpolate_ground_truth_locked(self, target_ns):
        if not self._ground_truth:
            return None
        first_ns = self._stamp_ns(self._ground_truth[0].header.stamp)
        last_ns = self._stamp_ns(self._ground_truth[-1].header.stamp)
        if target_ns < first_ns or target_ns > last_ns:
            return None
        right_index = len(self._ground_truth) - 1
        while (
            right_index > 0
            and self._stamp_ns(self._ground_truth[right_index - 1].header.stamp)
            >= target_ns
        ):
            right_index -= 1
        right = self._ground_truth[right_index]
        right_ns = self._stamp_ns(right.header.stamp)
        if right_ns == target_ns or right_index == 0:
            result = copy.deepcopy(right)
            result.header.stamp = rospy.Time.from_sec(target_ns * 1.0e-9)
            result.header.frame_id = self.world_frame
            return result
        left = self._ground_truth[right_index - 1]
        left_ns = self._stamp_ns(left.header.stamp)
        ratio = float(target_ns - left_ns) / float(right_ns - left_ns)
        result = Odometry()
        result.header.stamp = rospy.Time.from_sec(target_ns * 1.0e-9)
        result.header.frame_id = self.world_frame
        result.child_frame_id = left.child_frame_id
        for field in ("x", "y", "z"):
            a = getattr(left.pose.pose.position, field)
            b = getattr(right.pose.pose.position, field)
            setattr(result.pose.pose.position, field, a + ratio * (b - a))
            a = getattr(left.twist.twist.linear, field)
            b = getattr(right.twist.twist.linear, field)
            setattr(result.twist.twist.linear, field, a + ratio * (b - a))
        quaternion = self._slerp(
            left.pose.pose.orientation, right.pose.pose.orientation, ratio
        )
        (
            result.pose.pose.orientation.x,
            result.pose.pose.orientation.y,
            result.pose.pose.orientation.z,
            result.pose.pose.orientation.w,
        ) = quaternion
        return result

    def _ground_truth_callback(self, message):
        stamp_ns = self._stamp_ns(message.header.stamp)
        ready = []
        with self._lock:
            if self._ground_truth and stamp_ns < self._stamp_ns(
                self._ground_truth[-1].header.stamp
            ):
                self._ground_truth.clear()
                self._pending_epochs.clear()
            self._ground_truth.append(copy.deepcopy(message))
            oldest_ns = stamp_ns - self.gt_buffer_history_ns
            while len(self._ground_truth) > 2 and (
                len(self._ground_truth) > self.max_gt_buffer
                or self._stamp_ns(self._ground_truth[1].header.stamp) < oldest_ns
            ):
                self._ground_truth.popleft()
            for target_ns, pair in list(self._pending_epochs.items()):
                aligned = self._interpolate_ground_truth_locked(target_ns)
                if aligned is not None:
                    ready.append((pair[0], pair[1], aligned))
                    del self._pending_epochs[target_ns]
                elif target_ns < self._stamp_ns(self._ground_truth[0].header.stamp):
                    del self._pending_epochs[target_ns]
        for fused, status, aligned in ready:
            self._publish_epoch(fused, status, aligned)

    def _fused_callback(self, message):
        self._pair_message(message, True)

    def _integrity_callback(self, message):
        self._pair_message(message, False)

    def _pair_message(self, message, is_odometry):
        stamp_ns = self._stamp_ns(message.header.stamp)
        ready = None
        with self._lock:
            own = self._pending_odometry if is_odometry else self._pending_integrity
            other = self._pending_integrity if is_odometry else self._pending_odometry
            own[stamp_ns] = message
            match = other.pop(stamp_ns, None)
            if match is not None:
                own_message = own.pop(stamp_ns)
                fused, status = (
                    (own_message, match) if is_odometry else (match, own_message)
                )
                aligned = self._interpolate_ground_truth_locked(stamp_ns)
                if aligned is None:
                    self._pending_epochs[stamp_ns] = (fused, status)
                else:
                    ready = (fused, status, aligned)
            self._trim_pending_locked()
        if ready is not None:
            self._publish_epoch(*ready)

    def _trim_pending_locked(self):
        for pending in (
            self._pending_odometry,
            self._pending_integrity,
            self._pending_epochs,
        ):
            while len(pending) > self.max_pending:
                pending.popitem(last=False)

    def _record_epoch_paths(self, fused, aligned):
        stamp_ns = self._stamp_ns(fused.header.stamp)
        with self._lock:
            if self._last_path_sample_ns is not None and stamp_ns < self._last_path_sample_ns:
                self._fused_poses.clear()
                self._aligned_gt_poses.clear()
                self._last_path_sample_ns = None
            if (
                self._last_path_sample_ns is not None
                and stamp_ns - self._last_path_sample_ns < self.sample_period_ns
            ):
                return
            self._last_path_sample_ns = stamp_ns
            self._fused_poses.append(self._pose_stamped(fused))
            self._aligned_gt_poses.append(self._pose_stamped(aligned))
            cutoff_ns = stamp_ns - self.path_history_ns
            while self._fused_poses and self._stamp_ns(
                self._fused_poses[0].header.stamp
            ) < cutoff_ns:
                self._fused_poses.popleft()
                self._aligned_gt_poses.popleft()
            self._latest_path_stamp = fused.header.stamp
            self._paths_dirty = True

    def _path_timer_callback(self, _event):
        with self._lock:
            if not self._paths_dirty:
                return
            fused = tuple(self._fused_poses)
            aligned = tuple(self._aligned_gt_poses)
            stamp = self._latest_path_stamp
            self._paths_dirty = False
        self._publish_path(self._fused_path_pub, fused, stamp)
        self._publish_path(self._gt_path_pub, aligned, stamp)

    def _publish_path(self, publisher, poses, stamp):
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.world_frame
        path.poses = list(poses)
        publisher.publish(path)

    def _publish_epoch(self, fused, status, aligned):
        fused = copy.deepcopy(fused)
        fused.header.stamp = status.header.stamp
        fused.header.frame_id = self.world_frame
        aligned.header.stamp = status.header.stamp
        aligned.header.frame_id = self.world_frame
        self._fused_epoch_pub.publish(fused)
        self._aligned_gt_pub.publish(aligned)
        self._publish_pl_markers(fused, status)
        self._record_epoch_paths(fused, aligned)

    @staticmethod
    def _color(availability):
        state = availability.upper()
        if state == "AVAILABLE":
            return (0.0, 0.9, 0.9)
        if state == "ALERT":
            return (1.0, 0.45, 0.0)
        return (1.0, 0.1, 0.1)

    def _base_marker(self, marker_id, namespace, marker_type, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.world_frame
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _delete_marker(self, marker_id, namespace, marker_type, stamp):
        marker = self._base_marker(marker_id, namespace, marker_type, stamp)
        marker.action = Marker.DELETE
        return marker

    @staticmethod
    def _set_position(marker, position):
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = position.z

    @staticmethod
    def _set_color(marker, rgb, alpha):
        marker.color.r, marker.color.g, marker.color.b = rgb
        marker.color.a = alpha

    @staticmethod
    def _format_number(value):
        return "%.3f" % value if math.isfinite(value) else "INF"

    def _publish_pl_markers(self, odometry, status):
        stamp = status.header.stamp
        position = odometry.pose.pose.position
        pl = tuple(float(value) for value in status.pl_xyz_m)
        hpl = float(status.hpl_m)
        vpl = float(status.vpl_m)
        drawable = all(
            math.isfinite(value)
            and value >= 0.0
            and value <= self.max_drawable_pl_m
            for value in pl + (hpl, vpl)
        )
        rgb = self._color(status.availability)
        markers = []
        if drawable:
            ellipsoid = self._base_marker(
                self.ELLIPSOID_ID, "pl_ellipsoid", Marker.SPHERE, stamp
            )
            self._set_position(ellipsoid, position)
            ellipsoid.scale.x, ellipsoid.scale.y, ellipsoid.scale.z = (
                2.0 * pl[0],
                2.0 * pl[1],
                2.0 * pl[2],
            )
            self._set_color(ellipsoid, rgb, 0.18)
            markers.append(ellipsoid)

            ring = self._base_marker(
                self.HPL_RING_ID, "hpl_ring", Marker.LINE_STRIP, stamp
            )
            self._set_position(ring, position)
            ring.scale.x = 0.05
            self._set_color(ring, rgb, 0.95)
            for index in range(self.ring_segments + 1):
                angle = 2.0 * math.pi * index / self.ring_segments
                ring.points.append(
                    Point(hpl * math.cos(angle), hpl * math.sin(angle), 0.0)
                )
            markers.append(ring)

            vertical = self._base_marker(
                self.VPL_LINE_ID, "vpl_line", Marker.LINE_LIST, stamp
            )
            self._set_position(vertical, position)
            vertical.scale.x = 0.07
            self._set_color(vertical, rgb, 0.95)
            vertical.points = [Point(0.0, 0.0, -vpl), Point(0.0, 0.0, vpl)]
            markers.append(vertical)
            text_height = max(pl[2], vpl) + 0.45
        else:
            markers.extend(
                [
                    self._delete_marker(
                        self.ELLIPSOID_ID, "pl_ellipsoid", Marker.SPHERE, stamp
                    ),
                    self._delete_marker(
                        self.HPL_RING_ID, "hpl_ring", Marker.LINE_STRIP, stamp
                    ),
                    self._delete_marker(
                        self.VPL_LINE_ID, "vpl_line", Marker.LINE_LIST, stamp
                    ),
                ]
            )
            text_height = 0.8

        text = self._base_marker(
            self.STATUS_TEXT_ID, "integrity_status", Marker.TEXT_VIEW_FACING, stamp
        )
        self._set_position(text, position)
        text.pose.position.z += text_height
        text.scale.z = 0.32
        self._set_color(text, rgb, 1.0)
        numeric_label = (
            "HPL=%sm  VPL=%sm" % (self._format_number(hpl), self._format_number(vpl))
            if drawable
            else "PL=INF/OUT-OF-DISPLAY-RANGE"
        )
        text.text = (
            "%s  %s\n%s  detector=%s  commit=%s\n"
            "lag=%.1f ms  queue depth=%d\nT=%.3f / %.3f"
            % (
                status.availability.upper(),
                status.scope_label,
                numeric_label,
                "PASS" if status.detector_passed else "FAIL",
                "YES" if status.batch_committed else "NO",
                status.sensor_to_publish_ms,
                status.queue_depth,
                status.statistic,
                status.threshold,
            )
        )
        markers.append(text)
        self._marker_pub.publish(MarkerArray(markers=markers))


if __name__ == "__main__":
    rospy.init_node("realtime_integrity_viz")
    RealtimeIntegrityViz()
    rospy.spin()
