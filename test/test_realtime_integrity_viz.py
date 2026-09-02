#!/usr/bin/env python3
"""ROS integration tests for the realtime integrity RViz bridge."""

import math
import threading
import time
import unittest

import rospy
import rostest
from nav_msgs.msg import Odometry, Path
from uwb_imu_pl.msg import IntegrityStatus
from visualization_msgs.msg import Marker, MarkerArray


class RealtimeIntegrityVizTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self.gt_path = None
        self.fused_path = None
        self.markers = None
        self.aligned_ground_truth = None
        self.fused_epoch = None
        self.gt_pub = rospy.Publisher(
            "/test/viz/ground_truth", Odometry, queue_size=10
        )
        self.fused_pub = rospy.Publisher(
            "/test/viz/fused_odometry", Odometry, queue_size=10
        )
        self.integrity_pub = rospy.Publisher(
            "/test/viz/integrity", IntegrityStatus, queue_size=10
        )
        self.gt_sub = rospy.Subscriber(
            "/test/viz/ground_truth_path", Path, self._set_gt_path
        )
        self.fused_sub = rospy.Subscriber(
            "/test/viz/fused_path", Path, self._set_fused_path
        )
        self.marker_sub = rospy.Subscriber(
            "/test/viz/pl_markers", MarkerArray, self._set_markers
        )
        self.aligned_gt_sub = rospy.Subscriber(
            "/test/viz/aligned_ground_truth", Odometry, self._set_aligned_gt
        )
        self.fused_epoch_sub = rospy.Subscriber(
            "/test/viz/fused_epoch", Odometry, self._set_fused_epoch
        )
        self.assertTrue(
            self._wait_for(
                lambda: self.gt_pub.get_num_connections() > 0
                and self.fused_pub.get_num_connections() > 0
                and self.integrity_pub.get_num_connections() > 0
            ),
            "visualization node did not subscribe to all inputs",
        )

    def _set_gt_path(self, message):
        with self._lock:
            self.gt_path = message

    def _set_fused_path(self, message):
        with self._lock:
            self.fused_path = message

    def _set_markers(self, message):
        with self._lock:
            self.markers = message

    def _set_aligned_gt(self, message):
        with self._lock:
            self.aligned_ground_truth = message

    def _set_fused_epoch(self, message):
        with self._lock:
            self.fused_epoch = message

    def _wait_for(self, predicate, timeout=5.0):
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            with self._lock:
                if predicate():
                    return True
            rospy.sleep(0.02)
        return False

    @staticmethod
    def _odometry(stamp_s, x=1.0, y=2.0, z=3.0):
        message = Odometry()
        message.header.stamp = rospy.Time.from_sec(stamp_s)
        message.header.frame_id = "input_frame"
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.position.z = z
        message.pose.pose.orientation.w = 1.0
        return message

    @staticmethod
    def _integrity(stamp_s, availability="AVAILABLE", finite=True):
        message = IntegrityStatus()
        message.header.stamp = rospy.Time.from_sec(stamp_s)
        message.header.frame_id = "input_frame"
        message.scope_label = "FORMAL_LOCAL_CURRENT_FAULT_ONLY"
        message.availability = availability
        message.detector_passed = availability == "AVAILABLE"
        message.batch_committed = availability == "AVAILABLE"
        message.statistic = 2.5
        message.threshold = 10.0
        message.dof = 5
        message.measurement_model_valid = finite
        message.queue_depth = 7
        message.sensor_to_process_ms = 12.0
        message.sensor_to_publish_ms = 18.5
        if finite:
            message.pl_xyz_m = [1.0, 2.0, 3.0]
            message.hpl_m = 4.0
            message.vpl_m = 3.0
        else:
            message.pl_xyz_m = [math.inf, math.inf, math.inf]
            message.hpl_m = math.inf
            message.vpl_m = math.inf
        return message

    @staticmethod
    def _by_id(markers):
        return {marker.id: marker for marker in markers.markers}

    def test_paths_timestamp_pairing_and_pl_markers(self):
        # Ground truth brackets the estimator epoch; the displayed truth must
        # be interpolated to exactly the IntegrityStatus timestamp.
        self.gt_pub.publish(self._odometry(1.9, x=19.0))
        self.gt_pub.publish(self._odometry(2.1, x=21.0))
        rospy.sleep(0.05)

        # Integrity deliberately arrives first. Markers must wait for the exact
        # same timestamp rather than using a stale/latest fused pose.
        self.integrity_pub.publish(self._integrity(2.0))
        rospy.sleep(0.05)
        self.fused_pub.publish(self._odometry(2.0, x=7.0, y=8.0, z=9.0))
        self.assertTrue(
            self._wait_for(
                lambda: self.markers is not None
                and self.aligned_ground_truth is not None
                and self.fused_epoch is not None
                and self._by_id(self.markers).get(0) is not None
                and self._by_id(self.markers)[0].action == Marker.ADD
            )
        )
        self.assertEqual(
            self.aligned_ground_truth.header.stamp, rospy.Time.from_sec(2.0)
        )
        self.assertEqual(self.fused_epoch.header.stamp, rospy.Time.from_sec(2.0))
        self.assertAlmostEqual(
            self.aligned_ground_truth.pose.pose.position.x, 20.0, places=6
        )

        markers = self._by_id(self.markers)
        ellipsoid = markers[0]
        self.assertAlmostEqual(ellipsoid.pose.position.x, 7.0)
        self.assertAlmostEqual(ellipsoid.pose.position.y, 8.0)
        self.assertAlmostEqual(ellipsoid.pose.position.z, 9.0)
        self.assertAlmostEqual(ellipsoid.scale.x, 2.0)
        self.assertAlmostEqual(ellipsoid.scale.y, 4.0)
        self.assertAlmostEqual(ellipsoid.scale.z, 6.0)
        self.assertAlmostEqual(ellipsoid.color.g, 0.9)
        self.assertAlmostEqual(ellipsoid.color.b, 0.9)
        self.assertAlmostEqual(markers[1].points[0].x, 4.0)
        self.assertAlmostEqual(markers[1].points[0].y, 0.0)
        self.assertAlmostEqual(markers[2].points[0].z, -3.0)
        self.assertAlmostEqual(markers[2].points[1].z, 3.0)
        self.assertIn("HPL=4.000m", markers[3].text)
        self.assertIn("lag=18.5 ms", markers[3].text)
        self.assertIn("queue depth=7", markers[3].text)

        self.assertTrue(
            self._wait_for(
                lambda: self.gt_path is not None
                and self.fused_path is not None
                and len(self.gt_path.poses) == 1
                and len(self.fused_path.poses) == 1
            )
        )
        self.assertEqual(self.gt_path.header.frame_id, "world")
        self.assertEqual(self.fused_path.header.frame_id, "world")

        # ALERT remains drawable and changes the protection volume to orange.
        self.fused_pub.publish(self._odometry(2.1))
        self.integrity_pub.publish(self._integrity(2.1, availability="ALERT"))
        self.assertTrue(
            self._wait_for(
                lambda: self.markers is not None
                and self._by_id(self.markers)[0].action == Marker.ADD
                and self._by_id(self.markers)[0].color.r > 0.99
                and self._by_id(self.markers)[0].color.g > 0.4
            )
        )

        # Infinite PL must explicitly delete every prior finite protection
        # marker, while retaining a red status label.
        self.fused_pub.publish(self._odometry(2.2))
        self.gt_pub.publish(self._odometry(2.3, x=23.0))
        self.integrity_pub.publish(
            self._integrity(2.2, availability="UNAVAILABLE", finite=False)
        )
        self.assertTrue(
            self._wait_for(
                lambda: self.markers is not None
                and all(
                    self._by_id(self.markers)[marker_id].action == Marker.DELETE
                    for marker_id in (0, 1, 2)
                )
                and "PL=INF" in self._by_id(self.markers)[3].text
            )
        )
        self.assertAlmostEqual(self._by_id(self.markers)[3].color.r, 1.0)


if __name__ == "__main__":
    rospy.init_node("test_realtime_integrity_viz")
    rostest.rosrun(
        "uwb_imu_pl", "realtime_integrity_viz", RealtimeIntegrityVizTest
    )
