#!/usr/bin/env python3
"""
Quadrotor trajectory: takeoff → 3 circles → return → land.

Phases:
  1. Takeoff:  (0, 0, 0) → (0, 0, 2.0)  over 3 s
  2. Hold:     hover at (0, 0, 2.0)      for 2 s
  3. Circle:   radius 3.0 m, period 20 s, 3 loops (60 s total)
  4. Return:   fly back to (0, 0, 2.0)   over 5 s
  5. Land:     descend to (0, 0, 0.0)    over 3 s

Publishes: /position_cmd (uwb_imu_pl/PositionCommand)
"""

import math
import rospy
from uwb_imu_pl.msg import PositionCommand

PHASE_TAKEOFF = 0
PHASE_HOLD = 1
PHASE_CIRCLE = 2
PHASE_RETURN = 3
PHASE_LAND = 4


def main():
    rospy.init_node("trajectory_circle")

    takeoff_height = rospy.get_param("~takeoff_height", 2.0)
    takeoff_dur = rospy.get_param("~takeoff_duration", 3.0)
    hold_dur = rospy.get_param("~hold_duration", 2.0)
    circle_radius = rospy.get_param("~circle_radius", 3.0)
    circle_period = rospy.get_param("~circle_period", 20.0)
    num_circles = rospy.get_param("~num_circles", 3)
    return_dur = rospy.get_param("~return_duration", 5.0)
    land_dur = rospy.get_param("~land_duration", 3.0)
    rate = rospy.get_param("~rate", 100)

    pub = rospy.Publisher("/position_cmd", PositionCommand, queue_size=10)
    r = rospy.Rate(rate)

    rospy.sleep(1.0)

    start_time = rospy.Time.now().to_sec()
    phase = PHASE_TAKEOFF
    phase_start = start_time

    rospy.loginfo("=== Quadrotor Trajectory ===")
    rospy.loginfo("  Takeoff:  (0,0,0) -> (0,0,%.1f) in %.1fs", takeoff_height, takeoff_dur)
    rospy.loginfo("  Hold:     %.1fs", hold_dur)
    rospy.loginfo("  Circle:   r=%.1fm, T=%.1fs × %d loops", circle_radius, circle_period, num_circles)
    rospy.loginfo("  Return:   to origin in %.1fs", return_dur)
    rospy.loginfo("  Land:     to ground in %.1fs", land_dur)

    circle_dur = circle_period * num_circles
    circle_omega = 2.0 * math.pi / circle_period

    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        t = now - phase_start

        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "world"
        cmd.trajectory_id = 0
        cmd.yaw = 0.0
        cmd.yaw_dot = 0.0
        cmd.kx = [0.0, 0.0, 0.0]
        cmd.kv = [0.0, 0.0, 0.0]

        if phase == PHASE_TAKEOFF:
            if t >= takeoff_dur:
                phase = PHASE_HOLD
                phase_start = now
                rospy.loginfo(">>> Hold phase")
            else:
                s = t / takeoff_dur
                z = takeoff_height * (3 * s**2 - 2 * s**3)
                cmd.position.x = 0.0
                cmd.position.y = 0.0
                cmd.position.z = z

        elif phase == PHASE_HOLD:
            if t >= hold_dur:
                phase = PHASE_CIRCLE
                phase_start = now
                rospy.loginfo(">>> Circle phase (%d loops)", num_circles)
            cmd.position.x = 0.0
            cmd.position.y = 0.0
            cmd.position.z = takeoff_height

        elif phase == PHASE_CIRCLE:
            if t >= circle_dur:
                phase = PHASE_RETURN
                phase_start = now
                rospy.loginfo(">>> Return to origin")
            else:
                angle = circle_omega * t
                cmd.position.x = circle_radius * math.cos(angle)
                cmd.position.y = circle_radius * math.sin(angle)
                cmd.position.z = takeoff_height
                cmd.velocity.x = -circle_radius * circle_omega * math.sin(angle)
                cmd.velocity.y = circle_radius * circle_omega * math.cos(angle)
                cmd.yaw = angle + math.pi / 2.0
                cmd.yaw_dot = circle_omega

        elif phase == PHASE_RETURN:
            if t >= return_dur:
                phase = PHASE_LAND
                phase_start = now
                rospy.loginfo(">>> Landing")
            else:
                s = t / return_dur
                # Smooth step from current position to origin
                x = circle_radius * (1.0 - (3 * s**2 - 2 * s**3))
                cmd.position.x = x * math.cos(circle_omega * circle_dur)
                cmd.position.y = x * math.sin(circle_omega * circle_dur)
                cmd.position.z = takeoff_height

        elif phase == PHASE_LAND:
            if t >= land_dur:
                rospy.loginfo(">>> Landed. Holding at origin.")
                cmd.position.x = 0.0
                cmd.position.y = 0.0
                cmd.position.z = 0.0
            else:
                s = t / land_dur
                z = takeoff_height * (1.0 - (3 * s**2 - 2 * s**3))
                cmd.position.x = 0.0
                cmd.position.y = 0.0
                cmd.position.z = z

        pub.publish(cmd)
        r.sleep()


if __name__ == "__main__":
    main()
