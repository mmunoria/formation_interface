
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CommandNode(Node):

    def __init__(self):
        super().__init__("command_node")
        self.declare_parameter("drone_namespaces", [""])
        namespaces = self.get_parameter("drone_namespaces").value
       
        self.namespaces = list(namespaces) if namespaces else [""]

        self._cmd_pubs = {}
        for ns in self.namespaces:
            topic = f"/{ns}/command" if ns else "/command"
            self._cmd_pubs[topic] = self.create_publisher(String, topic, 10)

        self.get_logger().info(
            "command_node ready, broadcasting to: " + ", ".join(self._cmd_pubs))

    def broadcast(self, text):
        """Publish ``text`` as-is to every configured drone's command topic."""
        msg = String()
        msg.data = text
        for topic, pub in self._cmd_pubs.items():
            pub.publish(msg)
        self.get_logger().info(f"sent {text!r} to {len(self._cmd_pubs)} topic(s)")


def _run_menu(node):
    print("=== Drone Command Node ===")
    print("Broadcasting to: " + ", ".join(node._cmd_pubs))
    print()
    while rclpy.ok():
        print("  1) hover")
        print("  2) land")
        print("  3) custom command")
        print("  q) quit")
        try:
            choice = input("Select: ").strip().lower()
        except EOFError:
            break

        if choice in ("q", "quit", "exit"):
            break
        elif choice in ("1", "hover"):
            node.broadcast("hover")
        elif choice in ("2", "land"):
            node.broadcast("land")
        elif choice in ("3", "custom"):
            try:
                text = input("Enter custom command string: ").strip()
            except EOFError:
                break
            if text:
                node.broadcast(text)
            else:
                print("(empty command, not sent)")
        else:
            print(f"Unrecognized option: {choice!r}")
        print()


def main(args=None):
    rclpy.init(args=args)
    node = CommandNode()
    try:
        _run_menu(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
