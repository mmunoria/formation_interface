"""Simulated OptiTrack traffic, for dry-running the monitor GUI with no
hardware - parallel in spirit to :class:`drone_commander.MockCommander`'s
``backend:=sim``.

Publishes ``DronePose`` (each simulated drone wanders randomly inside the
configured rectangle) and periodic ``DroneActive`` heartbeats for a
configurable number of drones, and every ``toggle_period`` seconds flips one
drone's active state - so the whole pose -> registry -> map -> planner
pipeline, including drones joining/leaving, can be watched end to end without
OptiTrack or PX4.
"""

import math
import random

import rclpy
from rclpy.node import Node

from formation_interfaces.msg import DroneActive, DronePose


class MockOptitrackNode(Node):
    def __init__(self):
        super().__init__("mock_optitrack")
        self.declare_parameter("num_drones", 3)
        self.declare_parameter("map_xmin", -5.0)
        self.declare_parameter("map_xmax", 5.0)
        self.declare_parameter("map_ymin", -5.0)
        self.declare_parameter("map_ymax", 5.0)
        self.declare_parameter("rate", 10.0)            # Hz, pose publish rate
        self.declare_parameter("speed", 0.5)             # m/s wander speed
        self.declare_parameter("toggle_period", 15.0)    # s between active/inactive flips

        g = self.get_parameter
        self.n = int(g("num_drones").value)
        self.xmin, self.xmax = float(g("map_xmin").value), float(g("map_xmax").value)
        self.ymin, self.ymax = float(g("map_ymin").value), float(g("map_ymax").value)
        self.speed = float(g("speed").value)
        rate = float(g("rate").value)
        self.dt = 1.0 / rate

        self._pose_pub = self.create_publisher(DronePose, "/optitrack/drone_pose", 20)
        self._active_pub = self.create_publisher(DroneActive, "/optitrack/drone_active", 20)

        self._pos = [[random.uniform(self.xmin, self.xmax),
                      random.uniform(self.ymin, self.ymax)] for _ in range(self.n)]
        self._heading = [random.uniform(-math.pi, math.pi) for _ in range(self.n)]
        self._active = [True] * self.n

        self.create_timer(self.dt, self._step)
        self.create_timer(0.5, self._publish_active)          # heartbeat
        self.create_timer(float(g("toggle_period").value), self._toggle_one)

        self.get_logger().info(f"mock_optitrack: simulating {self.n} drones")

    def _step(self):
        for i in range(self.n):
            if not self._active[i]:
                continue
            if random.random() < 0.03:
                self._heading[i] += random.uniform(-1.0, 1.0)

            x, y = self._pos[i]
            nx = x + self.speed * self.dt * math.cos(self._heading[i])
            ny = y + self.speed * self.dt * math.sin(self._heading[i])
            if not (self.xmin <= nx <= self.xmax):
                self._heading[i] = math.pi - self._heading[i]
                nx = min(max(nx, self.xmin), self.xmax)
            if not (self.ymin <= ny <= self.ymax):
                self._heading[i] = -self._heading[i]
                ny = min(max(ny, self.ymin), self.ymax)
            self._pos[i] = [nx, ny]
            self._publish_pose(i, nx, ny, self._heading[i])

    def _publish_pose(self, drone_id, x, y, heading):
        msg = DronePose()
        msg.drone_id = drone_id
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        half = heading / 2.0
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)
        self._pose_pub.publish(msg)

    def _publish_active(self):
        for i in range(self.n):
            msg = DroneActive()
            msg.drone_id = i
            msg.active = self._active[i]
            self._active_pub.publish(msg)

    def _toggle_one(self):
        if self.n == 0:
            return
        i = random.randrange(self.n)
        self._active[i] = not self._active[i]
        state = "active" if self._active[i] else "inactive"
        self.get_logger().info(f"mock_optitrack: drone {i} -> {state}")


def main(args=None):
    rclpy.init(args=args)
    node = MockOptitrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
