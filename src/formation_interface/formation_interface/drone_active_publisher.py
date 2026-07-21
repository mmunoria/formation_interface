
import rclpy
from rclpy.node import Node

from formation_interfaces.msg import DroneActive


class DroneActivePublisherNode(Node):
    def __init__(self):
        super().__init__("drone_active_publisher")
        self.declare_parameter("drone_id", 0)
        self.declare_parameter("active_rate", 2.0)   # Hz, heartbeat publish rate

        g = self.get_parameter
        self.drone_id = int(g("drone_id").value)
        rate = float(g("active_rate").value)

        self._pub = self.create_publisher(DroneActive, "/optitrack/drone_active", 20)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f"drone_active_publisher: drone {self.drone_id} active heartbeat @ {rate} Hz")

    def _publish(self):
        msg = DroneActive()
        msg.drone_id = self.drone_id
        msg.active = True
        self._pub.publish(msg)


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
