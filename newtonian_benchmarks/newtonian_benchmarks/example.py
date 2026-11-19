
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from sensor_msgs.msg import JointState

from newtonian_msgs.msg import BaseCommand, JointCommand


NAMES = [f"{leg}_{joint}_joint" for leg in ("FR", "FL", "RR", "RL") for joint in ("hip", "thigh", "calf")]
HOME = [0.0, 0.9, -1.8] * 4


class StandController(Node):
    """
    Controller that holds the stand pose.
    """

    def __init__(self) -> None:
        """
        Reads the parameters and creates the topics.
        """
        super().__init__("example")

        # parameters
        self.rate = self.declare_parameter("rate", 100.0).value
        self.kp = self.declare_parameter("kp", 60.0).value
        self.kd = self.declare_parameter("kd", 3.0).value
        self.busy = self.declare_parameter("busy_ms", 0.0).value / 1000.0

        self.cmd: BaseCommand | None = None
        self._next = 0.0

        # topics
        self.pub = self.create_publisher(JointCommand, "/go2/control", QoSProfile(depth=10))
        self.create_subscription(BaseCommand, "/go2/base_command", self._on_command, QoSProfile(depth=5))
        self.create_subscription(JointState, "/go2/sensors/jointstate", self._on_joint_state, qos_profile_sensor_data)

    def _on_command(self, msg: BaseCommand) -> None:
        """
        Stores the last base command.
        """
        self.cmd = msg

    def _on_joint_state(self, msg: JointState) -> None:
        """
        Publishes one joint command for each period of sim time.
        """
        t = msg.header.stamp.sec + 1.0e-9 * msg.header.stamp.nanosec

        # the clock goes back when the simulator resets
        if t < self._next - 1.0:
            self._next = 0.0
        if t + 1.0e-9 < self._next:
            return
        self._next = t + 1.0 / self.rate

        self._burn()

        # stand pose
        out = JointCommand()
        out.header.stamp = msg.header.stamp
        out.name = NAMES
        out.position = list(HOME)
        out.velocity = [0.0] * 12
        out.effort = [0.0] * 12
        out.kp = [self.kp] * 12
        out.kd = [self.kd] * 12
        self.pub.publish(out)

    def _burn(self) -> None:
        """
        Burns 'busy_ms' of wall-clock time.
        """
        if self.busy <= 0.0:
            return

        end = time.perf_counter() + self.busy
        while time.perf_counter() < end:
            pass


def main() -> None:
    """
    Runs the node until shutdown.
    """
    rclpy.init()
    node = StandController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
