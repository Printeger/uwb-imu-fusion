#!/usr/bin/env python3
"""Deterministic straight/circle/figure-eight trajectory source for PL studies."""

import math

import rospy
from uwb_imu_pl.msg import PositionCommand


def smoothstep(value):
    return value * value * (3.0 - 2.0 * value)


def command_for(shape, elapsed, radius, omega, height):
    if shape == "straight":
        x = radius * math.sin(omega * elapsed)
        y = 0.0
        vx = radius * omega * math.cos(omega * elapsed)
        vy = 0.0
        ax = -radius * omega**2 * math.sin(omega * elapsed)
        ay = 0.0
    elif shape == "circle":
        x = radius * math.cos(omega * elapsed)
        y = radius * math.sin(omega * elapsed)
        vx = -radius * omega * math.sin(omega * elapsed)
        vy = radius * omega * math.cos(omega * elapsed)
        ax = -radius * omega**2 * math.cos(omega * elapsed)
        ay = -radius * omega**2 * math.sin(omega * elapsed)
    elif shape == "figure_eight":
        x = radius * math.sin(omega * elapsed)
        y = 0.5 * radius * math.sin(2.0 * omega * elapsed)
        vx = radius * omega * math.cos(omega * elapsed)
        vy = radius * omega * math.cos(2.0 * omega * elapsed)
        ax = -radius * omega**2 * math.sin(omega * elapsed)
        ay = -2.0 * radius * omega**2 * math.sin(2.0 * omega * elapsed)
    else:
        raise ValueError("trajectory must be straight, circle, or figure_eight")
    return x, y, height, vx, vy, 0.0, ax, ay, 0.0


def main():
    rospy.init_node("trajectory_realtime")
    shape = rospy.get_param("~trajectory", "figure_eight")
    radius = rospy.get_param("~radius", 3.0)
    period = rospy.get_param("~period", 20.0)
    height = rospy.get_param("~height", 2.0)
    takeoff_duration = rospy.get_param("~takeoff_duration", 3.0)
    rate_hz = rospy.get_param("~rate", 100.0)
    if shape not in ("straight", "circle", "figure_eight"):
        raise ValueError("unsupported trajectory: " + shape)
    if period <= 0.0 or rate_hz <= 0.0 or takeoff_duration <= 0.0:
        raise ValueError("period, rate, and takeoff_duration must be positive")
    omega = 2.0 * math.pi / period
    publisher = rospy.Publisher("/position_cmd", PositionCommand, queue_size=10)
    rate = rospy.Rate(rate_hz)
    start = rospy.Time.now().to_sec()
    rospy.loginfo("PL trajectory=%s radius=%.2f period=%.2f", shape, radius, period)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        elapsed = now.to_sec() - start
        if elapsed < takeoff_duration:
            state = (0.0, 0.0,
                     height * smoothstep(elapsed / takeoff_duration),
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            yaw = 0.0
            yaw_rate = 0.0
        else:
            state = command_for(shape, elapsed - takeoff_duration,
                                radius, omega, height)
            yaw = math.atan2(state[4], state[3])
            yaw_rate = omega
        message = PositionCommand()
        message.header.stamp = now
        message.header.frame_id = "world"
        message.trajectory_id = 0
        message.position.x, message.position.y, message.position.z = state[0:3]
        message.velocity.x, message.velocity.y, message.velocity.z = state[3:6]
        message.acceleration.x, message.acceleration.y, message.acceleration.z = state[6:9]
        message.yaw = yaw
        message.yaw_dot = yaw_rate
        message.kx = [0.0, 0.0, 0.0]
        message.kv = [0.0, 0.0, 0.0]
        publisher.publish(message)
        rate.sleep()


if __name__ == "__main__":
    main()
