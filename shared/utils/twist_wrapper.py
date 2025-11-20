"""Utility wrapper that abstracts Twist/TwistStamped selection."""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist, TwistStamped


class TwistWrapper:
    """Provide a unified API for stamped and non-stamped twist messages."""

    def __init__(self, use_stamped: bool = False) -> None:
        self.use_stamped = use_stamped
        if use_stamped:
            self._msg = TwistStamped()
            self._msg.header.stamp = rclpy.clock.Clock().now().to_msg()
            self._msg.header.frame_id = "base_link"
        else:
            self._msg = Twist()

    @property
    def linear(self):  # type: ignore[override]
        if self.use_stamped:
            return self._msg.twist.linear
        return self._msg.linear

    @property
    def angular(self):  # type: ignore[override]
        if self.use_stamped:
            return self._msg.twist.angular
        return self._msg.angular

    def get_message(self):
        if self.use_stamped:
            self._msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        return self._msg

    def reset(self) -> None:
        self.linear.x = 0.0
        self.linear.y = 0.0
        self.linear.z = 0.0
        self.angular.x = 0.0
        self.angular.y = 0.0
        self.angular.z = 0.0
