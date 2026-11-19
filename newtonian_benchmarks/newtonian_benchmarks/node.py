
from __future__ import annotations

from typing import Any

import argparse
import json
import os
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

from newtonian_msgs.msg import BaseCommand, JointCommand
from newtonian_srvs.srv import Configure

from newtonian_benchmarks.modules.plugins.profiles import Command
from newtonian_benchmarks.modules.runner.runner import BenchmarkRunner


class BenchmarkNode(Node):
    """
    ROS 2 node of the benchmark.
    """

    def __init__(self, topics: dict[str, str], command_topic: str, fall_tilt: float = 1.0, fall_height: float = 0.10) -> None:
        """
        Creates the subscriptions, the publisher, and the service client.
        """
        super().__init__("benchmark", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        # sim clock
        self.clock = 0.0
        self.clock_wall = 0.0
        self.clock_jumped_back = 0

        # recording
        self.odom: list[tuple] = []
        self.js: list[tuple] = []
        self.cmd_rx: list[tuple[float, float]] = []
        self.cmd_tx: list[tuple[float, float, float, float, float]] = []
        self.recording = False

        # command driver
        self.command: Command | None = None
        self.duration = 0.0
        self.rate = 50.0
        self._next_tx = 0.0
        self.done = False

        # fall
        self.fall_time: float | None = None
        self.fall_tilt = fall_tilt
        self.fall_height = fall_height

        # subscriptions
        self.create_subscription(Clock, "/clock", self._on_clock, QoSProfile(depth=1))
        self.create_subscription(Odometry, topics["ground_truth"], self._on_odometry, qos_profile_sensor_data)
        self.create_subscription(JointState, topics["joint_state"], self._on_joint_state, qos_profile_sensor_data)
        self.create_subscription(JointCommand, topics["joint_command"], self._on_command, QoSProfile(depth=50))

        # publisher and service client
        self.pub = self.create_publisher(BaseCommand, command_topic, QoSProfile(depth=5))
        self.configure_cli = self.create_client(Configure, "/newtonian/configure")

        # spin in a thread, so that the runner can wait on the records
        self.executor_ = SingleThreadedExecutor()
        self.executor_.add_node(self)
        self.thread = threading.Thread(target=self.executor_.spin, daemon=True)
        self.thread.start()

    def configure(self, request: dict[str, Any], timeout: float = 30.0) -> dict[str, Any]:
        """
        Calls the 'configure' service and returns its response.
        """
        if not self.configure_cli.wait_for_service(timeout_sec=timeout):
            raise TimeoutError("service /newtonian/configure not available")

        future = self.configure_cli.call_async(Configure.Request(request=json.dumps(request)))
        t = time.monotonic()
        while not future.done():
            if time.monotonic() - t > timeout:
                raise TimeoutError("configure timed out")
            time.sleep(0.002)

        r = future.result()
        if r is None:
            raise RuntimeError("configure returned no response")
        if not r.success:
            raise RuntimeError(f"configure failed: {r.message}")

        return json.loads(r.response) if r.response else {}

    def start(self, profile: dict[str, Any], duration: float) -> None:
        """
        Starts the command and the recording.
        """
        self.odom, self.js, self.cmd_rx, self.cmd_tx = [], [], [], []
        self.fall_time = None
        self.done = False
        self.clock_jumped_back = 0

        self.duration = duration
        self._next_tx = 0.0
        self.command = Command(p=profile)
        self.recording = True

    def stop(self) -> None:
        """
        Stops the command and the recording.
        """
        self.recording = False
        self.command = None

    def close(self) -> None:
        """
        Stops the executor and destroys the node.
        """
        self.executor_.shutdown()
        self.destroy_node()

    def _on_clock(self, msg: Clock) -> None:
        """
        Stores the clock and publishes the base command when it is due.
        """
        t = msg.clock.sec + 1.0e-9 * msg.clock.nanosec

        # the clock goes back when the simulator resets
        if t < self.clock - 0.5:
            self.clock_jumped_back += 1
        self.clock = t
        self.clock_wall = time.monotonic()

        if self.command is None or self.done:
            return
        if t + 1.0e-9 < self._next_tx:
            return

        # base command, at 'rate' Hz of sim time
        self._next_tx = max(self._next_tx + 1.0 / self.rate, t + 0.5 / self.rate)
        vx, vy, wz, h = self.command.at(t=t)

        m = BaseCommand()
        m.header.stamp = msg.clock
        m.vx, m.vy, m.yaw_rate, m.height = vx, vy, wz, h
        self.pub.publish(m)

        if self.recording:
            self.cmd_tx.append((t, vx, vy, wz, h))

        # the trial ends at the duration, or half a second after a fall
        if t >= self.duration or (self.fall_time is not None and t >= self.fall_time + 0.5):
            self.done = True

    def _on_odometry(self, msg: Odometry) -> None:
        """
        Records the ground-truth odometry and detects the fall.
        """
        if not self.recording:
            return

        t = msg.header.stamp.sec + 1.0e-9 * msg.header.stamp.nanosec
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        self.odom.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z))

        # fall: the base tilts too far or sinks too low
        if self.fall_time is None:
            tilt = float(np.arccos(np.clip(1.0 - 2.0 * (q.x * q.x + q.y * q.y), -1.0, 1.0)))
            if tilt > self.fall_tilt or p.z < self.fall_height:
                self.fall_time = t

    def _on_joint_state(self, msg: JointState) -> None:
        """
        Records the joint velocities and efforts.
        """
        if not self.recording:
            return

        t = msg.header.stamp.sec + 1.0e-9 * msg.header.stamp.nanosec
        self.js.append((t, list(msg.velocity), list(msg.effort)))

    def _on_command(self, msg: JointCommand) -> None:
        """
        Records the arrival of a joint command.
        """
        if not self.recording:
            return

        self.cmd_rx.append((self.clock, msg.header.stamp.sec + 1.0e-9 * msg.header.stamp.nanosec))


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    """
    Returns the command line arguments.
    """
    ap = argparse.ArgumentParser(description="newtonian robustness benchmark")
    ap.add_argument("--config", "--suite", dest="config", default="benchmarks", help="config file: name in config/ of this package, or a path. It holds the robot, the controller, and the suites")
    ap.add_argument("--only", help="comma-separated names of the suites of the file to run")
    ap.add_argument("--out", required=True)
    ap.add_argument("--reanalyze", action="store_true", help="compute the metrics of the traces in --out again, run nothing")
    ap.add_argument("--domain", type=int, default=60, help="ROS_DOMAIN_ID of the run (the simulator, the controller, and the benchmark node)")
    ap.add_argument("--limit", type=int, default=None, help="run only the first N trials")
    ap.add_argument("--rtf", type=float, default=None, help="real-time factor of the run, default of the config file")

    return ap.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    """
    Runs the benchmark.
    """
    args = parse_arguments(argv)
    out = Path(args.out)

    # new metrics or thresholds for the traces of an earlier run
    if args.reanalyze:
        BenchmarkRunner.reanalyze(out=out)
        return

    runner = BenchmarkRunner.from_yaml(path=args.config, out=out, domain=args.domain, rtf=args.rtf)

    # the domain is set before ROS 2 starts
    os.environ["ROS_DOMAIN_ID"] = str(runner.domain)
    rclpy.init()

    node = None
    try:
        thresholds = runner.suite["thresholds"]
        node = BenchmarkNode(
            topics=runner.profile["topics"],
            command_topic=runner.ctl["command"]["topic"],
            fall_tilt=float(thresholds.get("fall_tilt", 1.0)),
            fall_height=float(thresholds.get("fall_height", 0.10)),
        )
        runner.run(node=node, only=args.only.split(",") if args.only else None, limit=args.limit)
    finally:
        if node is not None:
            node.close()
        rclpy.try_shutdown()

    # the report after ROS 2 is shut down
    runner.write_report()


if __name__ == "__main__":
    main()
