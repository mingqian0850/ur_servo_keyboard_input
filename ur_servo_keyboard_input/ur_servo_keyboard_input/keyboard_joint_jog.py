#!/usr/bin/env python3
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType
from std_srvs.srv import SetBool


HELP = """
UR30 Servo Keyboard (JointJog)
--------------------------------
q/a : shoulder_pan +/-   w/s : shoulder_lift +/-
e/d : elbow +/-          r/f : wrist_1 +/-
t/g : wrist_2 +/-        y/h : wrist_3 +/-

z   : stop all joints
x   : decrease speed
c   : increase speed
CTRL+C to quit
"""


class KeyboardJointJog(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_joint_jog")

        self.declare_parameter("topic", "/servo_node/delta_joint_cmds")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("duration", 0.1)
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("base_speed", 0.2)
        self.declare_parameter("auto_prepare_servo", True)

        topic = self.get_parameter("topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.duration = float(self.get_parameter("duration").value)
        self.base_speed = float(self.get_parameter("base_speed").value)
        rate_hz = float(self.get_parameter("rate_hz").value)
        auto_prepare_servo = bool(self.get_parameter("auto_prepare_servo").value)

        self.joints = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.velocities = [0.0] * len(self.joints)

        self.keymap = {
            "q": (0, +1.0),
            "a": (0, -1.0),
            "w": (1, +1.0),
            "s": (1, -1.0),
            "e": (2, +1.0),
            "d": (2, -1.0),
            "r": (3, +1.0),
            "f": (3, -1.0),
            "t": (4, +1.0),
            "g": (4, -1.0),
            "y": (5, +1.0),
            "h": (5, -1.0),
        }

        self.pub = self.create_publisher(JointJog, topic, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)

        if auto_prepare_servo:
            self._prepare_servo_mode()

        print(HELP)
        self.get_logger().info(f"Publishing keyboard JointJog to {topic}")

    def _prepare_servo_mode(self) -> None:
        switch_cli = self.create_client(ServoCommandType, "/servo_node/switch_command_type")
        pause_cli = self.create_client(SetBool, "/servo_node/pause_servo")

        if switch_cli.wait_for_service(timeout_sec=2.0):
            req = ServoCommandType.Request()
            req.command_type = ServoCommandType.Request.JOINT_JOG
            future = switch_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if pause_cli.wait_for_service(timeout_sec=2.0):
            req = SetBool.Request()
            req.data = False
            future = pause_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

    def _read_key(self):
        dr, _, _ = select.select([sys.stdin], [], [], 0.0)
        if dr:
            return sys.stdin.read(1)
        return None

    def _tick(self) -> None:
        key = self._read_key()
        if key is not None:
            if key in self.keymap:
                idx, sign = self.keymap[key]
                self.velocities = [0.0] * len(self.joints)
                self.velocities[idx] = sign * self.base_speed
            elif key == "z":
                self.velocities = [0.0] * len(self.joints)
            elif key == "x":
                self.base_speed = max(0.01, self.base_speed - 0.05)
                self.get_logger().info(f"Speed = {self.base_speed:.2f}")
            elif key == "c":
                self.base_speed = min(2.0, self.base_speed + 0.05)
                self.get_logger().info(f"Speed = {self.base_speed:.2f}")

        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.joint_names = self.joints
        msg.displacements = []
        msg.velocities = self.velocities
        msg.duration = self.duration
        self.pub.publish(msg)


def main() -> None:
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    rclpy.init()
    node = KeyboardJointJog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
