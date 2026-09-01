#!/usr/bin/env python3
"""動的障害物 (地図に無い、動く物体) を決められた軌跡で動かす。

`make_env.py --dynamic` が出す `dynamic.json` を読み、各障害物を経路上で
往復させる。**位置は sim 時刻の関数として陽に決めて teleport する**。
速度指令で押すと物理の揺らぎで軌跡が実行ごとに変わり、位置推定の条件
(OFL / AMCL / 真値) を比較したときに障害物の違いが混ざってしまうためである。
リンクは kinematic なので teleport しても物理が破綻せず、衝突はする。

usage: obstacle_node.py --dynamic <dynamic.json> [--rate 10]
"""
import argparse
import json
import math
import sys

import rclpy
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node


def pose_at(path, speed, t):
    """折れ線 path 上を speed [m/s] で往復したときの時刻 t の (x, y, yaw)。"""
    segs = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
    lens = [math.dist(a, b) for a, b in segs]
    total = sum(lens)
    if total < 1e-9:
        return path[0][0], path[0][1], 0.0
    s = (speed * t) % (2 * total)
    back = s > total
    if back:
        s = 2 * total - s
    for (a, b), L in zip(segs, lens):
        if s <= L or L < 1e-9:
            u = s / L if L > 1e-9 else 0.0
            x = a[0] + u * (b[0] - a[0])
            y = a[1] + u * (b[1] - a[1])
            yaw = math.atan2(b[1] - a[1], b[0] - a[0])
            return x, y, (yaw + math.pi if back else yaw)
        s -= L
    x, y = path[-1]
    return x, y, 0.0


class Movers(Node):
    def __init__(self, dyn, rate):
        super().__init__('ofl_obstacle_mover')
        self.dyn = dyn
        self.pending = []
        self.cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        # 記録側が「今どこに居たか」を厳密に知れるように公開する
        # (記録側で時刻から再計算すると t0 の取り方でずれる)
        self.pub = self.create_publisher(PoseArray, '/dyn_obstacles', 10)
        self.create_timer(1.0 / rate, self.step)
        self.get_logger().info(f'moving {len(dyn)} obstacles at {rate} Hz')

    def step(self):
        if not self.cli.service_is_ready():
            return
        # **sim 時刻そのもの**で位置を決める。起動時刻を原点にすると、ノードが
        # 立ち上がるタイミングの揺らぎが障害物の位相差になり、位置推定の条件を
        # 変えて比べたときに障害物の違いが混ざる。
        t = self.get_clock().now().nanoseconds * 1e-9
        arr = PoseArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = 'map'
        for d in self.dyn:
            x, y, yaw = pose_at(d['path'], d['speed'], t)
            req = SetEntityState.Request()
            req.state.name = d['name']
            req.state.pose.position.x = x
            req.state.pose.position.y = y
            req.state.pose.position.z = d['size'][2] / 2.0
            req.state.pose.orientation.z = math.sin(yaw / 2)
            req.state.pose.orientation.w = math.cos(yaw / 2)
            req.state.reference_frame = 'world'
            self.pending.append(self.cli.call_async(req))
            q = Pose()
            q.position.x, q.position.y = x, y
            q.orientation.z, q.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            arr.poses.append(q)
        self.pub.publish(arr)
        # 完了した future を捨てないと際限なく溜まる
        self.pending = [f for f in self.pending if not f.done()]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dynamic', required=True)
    ap.add_argument('--rate', type=float, default=10.0)
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    dyn = json.load(open(a.dynamic))
    if not dyn:
        print('no dynamic obstacles; exiting')
        return
    rclpy.init(args=sys.argv)
    node = Movers(dyn, a.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
