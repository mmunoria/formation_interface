#!/usr/bin/env python3
import argparse
import math
import random

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool


class _DroneState:
    def __init__(self, drone_id: int, node: Node, pose_topic: str, hover):
        self.id = drone_id
        self.hover = hover                                  # (x, y, z)
        self.phase = random.uniform(0.0, 2.0 * math.pi)      # decorrelate wobble
        self.active_pub = node.create_publisher(
            Bool, f"/drone{drone_id}/active", 10)
        self.pose_pub = node.create_publisher(PoseStamped, pose_topic, 10)


class MultiDroneActivePublisher(Node):

    def __init__(self, drone_ids, rate: float, pose_topic_template: str,
                 spacing: float, hover_z: float, jitter: float,
                 yaw_jitter: float):
        super().__init__("mock_drones_active_publisher")
        self.jitter = jitter
        self.yaw_jitter = yaw_jitter
        self._t0 = self.get_clock().now()

        # Spread hover points out along x, centred on the origin, so drones
        # don't sit on top of each other - same spirit as formations.py's
        # 'line' generator, just for a static hover layout.
        n = len(drone_ids)
        self.drones = [
            _DroneState(
                drone_id, self,
                pose_topic_template.format(id=drone_id),
                ((i - (n - 1) / 2.0) * spacing, 0.0, hover_z))
            for i, drone_id in enumerate(drone_ids)
        ]

        self.create_timer(1.0 / rate, self._publish_all)
        topics = ", ".join(f"/drone{d.id}/active" for d in self.drones)
        self.get_logger().info(
            f"publishing active=True + hovering poses for {n} drone(s) "
            f"at {rate} Hz ({topics})")

    def _publish_all(self):
        elapsed = (self.get_clock().now() - self._t0).nanoseconds / 1e9
        now = self.get_clock().now().to_msg()
        for d in self.drones:
            d.active_pub.publish(Bool(data=True))

            x = d.hover[0] + random.gauss(0.0, self.jitter)
            y = d.hover[1] + random.gauss(0.0, self.jitter)
            z = (d.hover[2] + 0.01 * math.sin(0.3 * elapsed + d.phase)
                 + random.gauss(0.0, self.jitter * 0.5))
            yaw = self.yaw_jitter * math.sin(0.15 * elapsed + d.phase)

            msg = PoseStamped()
            msg.header.stamp = now
            msg.header.frame_id = "map"
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
            msg.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.orientation.w = math.cos(yaw / 2.0)
            d.pose_pub.publish(msg)


def _parse_drone_ids(args):
    if args.drone_ids:
        return [int(s) for s in args.drone_ids.split(",")]
    return list(range(1, args.num_drones + 1))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--num-drones", type=int, default=5,
                         help="number of drones, ids 1..N (default: 5, "
                              "matching config/drones.yaml)")
    parser.add_argument("--drone-ids", type=str, default=None,
                         help="comma-separated explicit drone ids, "
                              "overrides --num-drones (e.g. '1,3,4')")
    parser.add_argument("--pose-topic-template", type=str,
                         default="/vrpn_mocap/drone{id}/pose",
                         help="pose topic per drone, '{id}' is substituted "
                              "(default matches config/drones.yaml)")
    parser.add_argument("--rate", type=float, default=10.0,
                         help="publish rate in Hz (default: 10.0)")
    parser.add_argument("--spacing", type=float, default=1.0,
                         help="metres between adjacent drones' hover points "
                              "(default: 1.0)")
    parser.add_argument("--hover-z", type=float, default=1.0,
                         help="hover altitude in metres (default: 1.0)")
    parser.add_argument("--jitter", type=float, default=0.01,
                         help="position noise std dev in metres, simulating "
                              "mocap/hover noise (default: 0.01)")
    parser.add_argument("--yaw-jitter", type=float, default=0.03,
                         help="yaw wobble amplitude in radians (default: 0.03)")
    args, ros_args = parser.parse_known_args()

    drone_ids = _parse_drone_ids(args)

    rclpy.init(args=ros_args)
    node = MultiDroneActivePublisher(
        drone_ids, args.rate, args.pose_topic_template,
        args.spacing, args.hover_z, args.jitter, args.yaw_jitter)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
