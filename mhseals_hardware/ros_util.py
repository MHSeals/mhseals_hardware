"""ROS helpers shared by the hardware nodes."""

from geometry_msgs.msg import Twist
import rclpy


def twist_from(command):
    """Build a Twist from a (surge, sway, yaw) command."""
    message = Twist()
    message.linear.x, message.linear.y, message.angular.z = (float(value) for value in command)
    return message


def twist_values(twist):
    """Return the (surge, sway, yaw) components of a Twist."""
    return (twist.linear.x, twist.linear.y, twist.angular.z)


def run_node(node_factory, args=None):
    """Spin one node until shutdown; Ctrl+C exits quietly and cleanup always runs.

    Nodes that drive hardware put their fail-safe in ``destroy_node``, so it
    must run even when construction or spinning fails.
    """
    rclpy.init(args=args)
    node = None
    try:
        node = node_factory()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
