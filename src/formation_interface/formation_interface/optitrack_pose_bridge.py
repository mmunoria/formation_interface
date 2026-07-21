import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

from formation_interfaces.msg import DronePose


class OptitrackPoseBridgeNode(Node):
    def __init__(self):
        super().__init__("optitrack_pose_bridge")
        self.declare_parameter("drone_id", 1)
        self.declare_parameter("source_topic", "/optitrack_pose")

        g = self.get_parameter
        self.drone_id = int(g("drone_id").value)
        source_topic = str(g("source_topic").value)

        self._pub = self.create_publisher(DronePose, "/optitrack/drone_pose", 20)
        self.create_subscription(PoseStamped, source_topic, self._on_pose, 20)

        self.get_logger().info(
            f"optitrack_pose_bridge: {source_topic} -> drone_id {self.drone_id}")

    def _on_pose(self, msg):
        out = DronePose()
        out.drone_id = self.drone_id
        out.pose = msg
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OptitrackPoseBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
