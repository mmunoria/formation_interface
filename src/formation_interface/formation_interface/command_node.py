"""
    ros2 run formation_interface command_node
    ros2 run formation_interface command_node --ros-args \\
        -p drone_domains:="[1,2,3,4,5,6,7]" -p drone_namespaces:="['px4_1','px4_2','px4_3','px4_4','px4_5','px4_6','px4_7']"
"""

import time

import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from std_msgs.msg import String


class _DomainTarget:
    """One drone's publisher, isolated in its own rclpy Context/domain.

    ``domain_id=None`` means "whatever ROS_DOMAIN_ID this process was
    launched with" (rclpy's own default) rather than pinning to a specific
    domain - used for the legacy single-domain case.
    """

    def __init__(self, index, domain_id, namespace):
        self.domain_id = domain_id
        self.namespace = namespace
        self.topic = f"/{namespace}/command" if namespace else "/command"

        self.context = Context()
        rclpy.init(context=self.context, domain_id=domain_id)
        suffix = domain_id if domain_id is not None else "default"
        self.node = rclpy.create_node(
            f"command_node_{index}_d{suffix}", context=self.context)
        self.pub = self.node.create_publisher(String, self.topic, 10)

    def describe(self):
        domain = self.domain_id if self.domain_id is not None else "default"
        return f"{self.topic} (domain {domain})"

    def publish(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)


def _load_targets(args):
    """Read drone_domains / drone_namespaces via a throwaway bootstrap node
    (so --ros-args --params-file / -p work as usual), then build one
    _DomainTarget per drone. The bootstrap node is torn down immediately
    after - it only exists to read parameters."""
    rclpy.init(args=args)
    cfg_node = rclpy.create_node("command_node_config")
    # Declared by *type*, not by an empty-list default - an empty list can't
    # be type-inferred by rclpy on its own (it guesses BYTE_ARRAY and then
    # rejects any int list handed in from a params file).
    cfg_node.declare_parameter("drone_domains", Parameter.Type.INTEGER_ARRAY)
    cfg_node.declare_parameter("drone_namespaces", [""])
    # get_parameter_or, not get_parameter: an INTEGER_ARRAY-typed-but-unset
    # parameter (no override given -> legacy single-domain mode) raises
    # ParameterUninitializedException on a bare .get_parameter().
    domains_default = Parameter("drone_domains", Parameter.Type.INTEGER_ARRAY, [])
    domains = list(cfg_node.get_parameter_or("drone_domains", domains_default).value)
    namespaces = list(cfg_node.get_parameter("drone_namespaces").value)
    cfg_node.destroy_node()
    rclpy.shutdown()

    if not domains:
        # Legacy / single-domain: no per-drone domain switching.
        domains = [None] * len(namespaces or [""])
        namespaces = namespaces or [""]
    elif len(namespaces) != len(domains):
        if namespaces in ([], [""]):
            namespaces = [""] * len(domains)
        else:
            raise ValueError(
                "drone_domains and drone_namespaces must be the same length "
                f"(got {len(domains)} domains, {len(namespaces)} namespaces)")

    return [
        _DomainTarget(i, d, ns)
        for i, (d, ns) in enumerate(zip(domains, namespaces))
    ]


def _broadcast(targets, text):
    for target in targets:
        target.publish(text)
    print(f"sent {text!r} to {len(targets)} target(s)")


def _run_menu(targets):
    print("=== Drone Command Node ===")
    for target in targets:
        print(f"  -> {target.describe()}")
    print()
    while True:
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
            _broadcast(targets, "hover")
        elif choice in ("2", "land"):
            _broadcast(targets, "land")
        elif choice in ("3", "custom"):
            try:
                text = input("Enter custom command string: ").strip()
            except EOFError:
                break
            if text:
                _broadcast(targets, text)
            else:
                print("(empty command, not sent)")
        else:
            print(f"Unrecognized option: {choice!r}")
        print()


def main(args=None):
    targets = _load_targets(args)
    # DDS discovery across N freshly-created domain participants takes a
    # moment; without this, a command typed immediately after startup can
    # go out before any given domain's subscribers have been matched yet
    # and simply vanish (observed with sub-second gaps in testing).
    print(f"Waiting for {len(targets)} domain participant(s) to settle...")
    time.sleep(1.5)
    try:
        _run_menu(targets)
    except KeyboardInterrupt:
        pass
    finally:
        for target in targets:
            target.close()


if __name__ == "__main__":
    main()
