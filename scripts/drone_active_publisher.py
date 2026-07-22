#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class DroneActivePublisherNode(Node):
    def __init__(self):
        super().__init__("drone_active_publisher")
        self.declare_parameter("drone_id", 1)
        self.declare_parameter("active_rate", 2.0)   # Hz, heartbeat publish rate

        g = self.get_parameter
        self.drone_id = int(g("drone_id").value)
        rate = float(g("active_rate").value)

        self._pub = self.create_publisher(Bool, f"/drone{self.drone_id}/active", 20)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f"drone_active_publisher: drone {self.drone_id} active heartbeat @ {rate} Hz "
            f"on /drone{self.drone_id}/active")

    def _publish(self):
        self._pub.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = DroneActivePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
