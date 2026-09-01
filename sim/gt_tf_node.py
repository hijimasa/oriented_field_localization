#!/usr/bin/env python3
"""真値から map -> odom を出す「完全な位置推定」。

Nav2 閉ループの上限を測るための基準線。位置推定の誤差をゼロにしたときに
この航法スタックが何秒でどれだけの余裕を持って走れるかが分かるので、
OFL / AMCL の結果を「航法の限界」と「位置推定の限界」に切り分けられる。

REP-105: T_map_odom = T_map_base * T_odom_base^-1
"""
import math
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GtTf(Node):
    def __init__(self):
        super().__init__('gt_tf')
        self.declare_parameter('transform_tolerance', 0.2)
        self.tol = self.get_parameter('transform_tolerance').value
        self.odom = None
        self.br = TransformBroadcaster(self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos)
        self.create_subscription(Odometry, 'ground_truth', self.on_gt, qos)

    def on_odom(self, m):
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y,
                     yaw_of(m.pose.pose.orientation))

    def on_gt(self, m):
        if self.odom is None:
            return
        bx, by = m.pose.pose.position.x, m.pose.pose.position.y
        byaw = yaw_of(m.pose.pose.orientation)
        ox, oy, oyaw = self.odom
        # T_map_odom = T_map_base * T_odom_base^-1
        th = byaw - oyaw
        c, s = math.cos(th), math.sin(th)
        tx = bx - (c * ox - s * oy)
        ty = by - (s * ox + c * oy)
        t = TransformStamped()
        stamp = self.get_clock().now() + rclpy.duration.Duration(seconds=self.tol)
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.rotation.z = math.sin(th / 2)
        t.transform.rotation.w = math.cos(th / 2)
        self.br.sendTransform(t)


def main():
    rclpy.init(args=sys.argv)
    rclpy.spin(GtTf())


if __name__ == '__main__':
    main()
