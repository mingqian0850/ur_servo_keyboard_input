#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType
from std_srvs.srv import SetBool


class JointJogPublisher(Node):
    def __init__(self) -> None:
        super().__init__("joint_jog_publisher")

        self.declare_parameter("topic", "/servo_node/delta_joint_cmds")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("joint_name", "shoulder_pan_joint")
        self.declare_parameter("velocity", 0.2)
        self.declare_parameter("duration", 0.1)
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("auto_prepare_servo", True)

        topic = self.get_parameter("topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.joint_name = self.get_parameter("joint_name").value
        self.velocity = float(self.get_parameter("velocity").value)
        self.duration = float(self.get_parameter("duration").value)
        rate_hz = float(self.get_parameter("rate_hz").value)
        auto_prepare_servo = bool(self.get_parameter("auto_prepare_servo").value)

        self.pub = self.create_publisher(JointJog, topic, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self._publish_once)

        if auto_prepare_servo:
            self._prepare_servo_mode()

        self.get_logger().info(
            f"Publishing JointJog to {topic}: joint={self.joint_name}, velocity={self.velocity}, rate={rate_hz}Hz"
        )

    def _prepare_servo_mode(self) -> None:
        switch_cli = self.create_client(ServoCommandType, "/servo_node/switch_command_type")
        pause_cli = self.create_client(SetBool, "/servo_node/pause_servo")

        if switch_cli.wait_for_service(timeout_sec=2.0):
            req = ServoCommandType.Request()
            req.command_type = ServoCommandType.Request.JOINT_JOG
            future = switch_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        else:
            self.get_logger().warn("Service /servo_node/switch_command_type not available.")

        if pause_cli.wait_for_service(timeout_sec=2.0):
            req = SetBool.Request()
            req.data = False
            future = pause_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        else:
            self.get_logger().warn("Service /servo_node/pause_servo not available.")

    def _publish_once(self) -> None:
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.joint_names = [self.joint_name]
        msg.velocities = [self.velocity]
        msg.displacements = []
        msg.duration = self.duration
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = JointJogPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
