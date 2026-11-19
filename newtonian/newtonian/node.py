
from __future__ import annotations

from typing import Any

import collections
import json
import signal
import time

import numpy as np

import rclpy
import rclpy.executors
from rclpy.clock import Clock as RosClock
from rclpy.clock_type import ClockType
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from builtin_interfaces.msg import Time
from newtonian_srvs.srv import Configure
from rosgraph_msgs.msg import Clock
from std_srvs.srv import SetBool, Trigger

from newtonian.modules.params import Parameters
from newtonian.modules.physics.simulator import NewtonianSimulator
from newtonian.modules.plugins.sensors import Sensor


_ROS_PARAMETERS = ("use_sim_time", "start_type_description_service")
_RTF_WINDOW = 5.0
_RTF_TOLERANCE = 0.01


def _qos(item) -> QoSProfile:
    """
    Returns the QoS profile of the 'qos' key of 'item'.
    """
    if item.qos == "sensor_data":
        return qos_profile_sensor_data
    if item.qos == "default":
        return QoSProfile(depth=item.qos_depth)
    raise ValueError(f"'{item.name}' must be 'default' or 'sensor_data'")


class NewtonianNode(Node):
    """
    ROS 2 node that steps the simulator, publishes the sensors, and subscribes to the inputs.
    """

    def __init__(self) -> None:
        """
        Creates the clock publisher and the services, and builds the scene.
        """
        super().__init__("newtonian", allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.sim: NewtonianSimulator | None = None
        self.timer: Timer | None = None
        self.publishers_: dict[str, Publisher] = {}
        self.subscriptions_: list[Subscription] = []
        self.clock_pub = self.create_publisher(msg_type=Clock, topic="/clock", qos_profile=QoSProfile(depth=1))
        self.create_service(srv_type=Trigger, srv_name="~/reset", callback=self._on_reset)
        self.create_service(srv_type=Trigger, srv_name="~/rebuild", callback=self._on_rebuild)
        self.create_service(srv_type=SetBool, srv_name="~/pause", callback=self._on_pause)
        self.create_service(srv_type=Configure, srv_name="~/configure", callback=self._on_configure)
        self._rebuild()

    def _check_rtf(self, sim: NewtonianSimulator) -> None:
        """
        Logs a warning if the real-time factor differs from the target.
        """
        now = time.monotonic()
        if sim.frame != self._rtf_frame:
            self._rtf_window = None
        self._rtf_frame = sim.frame + 1
        if self._rtf_window is None:
            self._rtf_window = (now, sim.sim_time)
            return
        wall_start, sim_start = self._rtf_window
        if now - wall_start < _RTF_WINDOW:
            return
        rtf = (sim.sim_time - sim_start) / (now - wall_start)
        if abs(rtf - self.rtf) > _RTF_TOLERANCE * self.rtf:
            self.get_logger().warning(f"real-time factor {rtf:.2f} differs from the target {self.rtf:.2f}")
        self._rtf_window = (now, sim.sim_time)

    def _params(self) -> Parameters:
        """
        Returns the node parameters without the ROS parameters.
        """
        names = self.list_parameters([], 0).names
        flat = {n: p.value for n, p in zip(names, self.get_parameters(names)) if n not in _ROS_PARAMETERS}
        return Parameters(flat)

    def _teardown(self) -> None:
        """
        Destroys the timer, the publishers, the subscriptions, and the simulator.
        """
        if self.timer is not None:
            self.destroy_timer(self.timer)
        for subscription in self.subscriptions_:
            self.destroy_subscription(subscription)
        for publisher in self.publishers_.values():
            self.destroy_publisher(publisher)
        self.subscriptions_.clear()
        self.publishers_.clear()
        if self.sim is not None:
            self.sim.close()
            self.sim = None

    def _rebuild(self) -> None:
        """
        Builds a new simulator, and keeps the old one if the build fails.
        """
        p = self._params()
        old = self.sim
        viewer = old.viewer if old is not None else None
        try:
            sim = NewtonianSimulator(p=p, viewer=viewer)
        except Exception:
            if old is not None and viewer is not None and viewer.sim is not old:
                viewer.attach(sim=old)
            raise
        if old is not None and sim.viewer is viewer:
            old.viewer = None
        self._teardown()
        self.sim = sim

        for s in self.sim.sensors:
            self.publishers_[s.name] = self.create_publisher(s.msg_type, s.topic, _qos(s))

        for i in self.sim.inputs:
            self.subscriptions_.append(self.create_subscription(i.msg_type, i.topic, i.callback, _qos(i)))

        self.rtf = self.sim.real_time_factor
        self._render_next = 0.0
        self._rtf_window: tuple[float, float] | None = None
        self._rtf_frame = 0
        self._delay: dict[str, list[tuple[float, Any]]] = {}
        self._timing: collections.deque = collections.deque(maxlen=2000)
        self._step_ms: collections.deque = collections.deque(maxlen=2000)
        self._stale: dict[str, Any] = {}
        self._stale_clock: Time | None = None
        self._stale_due: dict[str, float] | None = None
        self.timer = self.create_timer(timer_period_sec=self.sim.frame_dt / self.rtf, callback=self._on_timer, clock=RosClock(clock_type=ClockType.STEADY_TIME))

    def _stamp(self, sim: NewtonianSimulator) -> Time:
        """
        Returns the simulation time of the current frame.
        """
        ns = round(sim.frame * 1.0e+9 / sim.frame_rate)
        return Time(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000)

    def _on_timer(self) -> None:
        """
        Steps one frame, publishes the clock and the sensors, and renders.
        """
        sim = self.sim
        if sim is None or not self.context.ok():
            return
        if sim.viewer is not None and not sim.viewer.is_running():
            self.get_logger().info("viewer closed, shutting down ...")
            rclpy.try_shutdown()
            return
        if sim.viewer is not None and sim.viewer.take_reset_request():
            self._rtf_window = None
            self._delay.clear()
            sim.reset()
        if sim.warmingup:
            self._rtf_window = None
            self._publish_stale(sim=sim)
            sim.warmup(budget=0.5 * sim.frame_dt / self.rtf)
        elif not sim.stepable():
            self._rtf_window = None
        else:
            self._stale_due = None
            self._check_rtf(sim=sim)

            tic = time.monotonic()
            sim.step()

            stamp = self._stamp(sim=sim)
            self._stale_clock = stamp
            self.clock_pub.publish(Clock(clock=stamp))
            for s in sim.sense():
                msg = s.to_msg(stamp=stamp)
                self._stale[s.name] = msg
                self._publish(sim=sim, sensor=s, msg=msg)
            self._publish_due(sim=sim)

            toc = time.monotonic()
            self._step_ms.append(1000.0 * (toc - tic))
            self._timing.append((toc, sim.sim_time))

        if not sim.warmingup and self._render_due(sim=sim):
            sim.render()

    def _render_due(self, sim: NewtonianSimulator) -> bool:
        """
        True if the viewer is due for a frame, which is at most 'viewer_rate' per wall-clock second.
        Every frame is drawn if the frame period is not shorter than the viewer period.
        """
        tick = sim.frame_dt / self.rtf
        period = 1.0 / sim.viewer_rate
        if period <= tick:
            return True
        now = time.monotonic()
        if now + 0.5 * tick < self._render_next:
            return False
        self._render_next = self._render_next + period if now - self._render_next < period else now + period
        return True

    def _on_configure(self, request: Configure.Request, response: Configure.Response) -> Configure.Response:
        """
        """
        if self.sim is None:
            response.success = False
            response.message = "no scene"
            return response

        try:
            loaded = json.loads(request.request) if request.request.strip() else {}
            out = self.sim.benchmarker.configure(request=loaded)

            if loaded.get("reset"):
                self._rtf_window = None
                self._delay.clear()
                self._timing.clear()
                self._step_ms.clear()

            if "rtf" in loaded:
                self._set_rtf(float(loaded["rtf"]))
            if loaded.get("timing"):
                out["timing"] = self._report_timing()

            response.success = True
            response.response = json.dumps(out)

        except Exception as e:
            response.success = False
            response.message = f"{type(e).__name__} : {e}"

        return response

    def _on_reset(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """
        Resets the simulator.
        """
        self._rtf_window = None
        if self.sim is None:
            response.success = False
            response.message = "no scene"
            return response
        self.sim.reset()
        self._delay.clear()
        response.success = True
        return response

    def _on_rebuild(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """
        Rebuilds the simulator from the current parameters.
        """
        try:
            self._rebuild()
            response.success = True
        except Exception as e:
            self.get_logger().error("rebuild failed")
            response.success = False
            response.message = str(e)
        return response

    def _on_pause(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        """
        Pauses or resumes stepping.
        """
        if self.sim is not None:
            self.sim.pause(is_paused=bool(request.data))
            response.message = "paused" if self.sim.is_paused else "running"
        response.success = self.sim is not None
        return response

    def _publish(self, sim: NewtonianSimulator, sensor: Sensor, msg: Any) -> None:
        """
        """
        is_ideal = sensor.latency == 0.0 and sensor.jitter == 0.0 and sensor.drop == 0.0

        if is_ideal and not self._delay.get(sensor.name):
            self.publishers_[sensor.name].publish(msg)
            return

        if sensor.drop > 0.0 and sim.rng.random() < sensor.drop:
            return

        delay = sensor.latency
        if sensor.jitter > 0.0:
            delay += float(sim.rng.uniform(-sensor.jitter, sensor.jitter))

        self._delay.setdefault(sensor.name, []).append((sim.sim_time + max(delay, 0.0), msg))

    def _publish_due(self, sim: NewtonianSimulator) -> None:
        """
        Publishes any delayed messages that are due.
        """
        tic = sim.sim_time + 1.0e-9

        for name, items in self._delay.items():
            if not items:
                continue

            items.sort(key=lambda item: item[0])
            while items and items[0][0] <= tic:
                self.publishers_[name].publish(items.pop(0)[1])

    def _publish_stale(self, sim: NewtonianSimulator) -> None:
        """
        Publishes the last clock and the last message of each sensor again
        """
        if self._stale_clock is None:
            return
        now = time.monotonic()
        periods = {s.name: (1.0 / s.rate if s.rate > 0.0 else sim.frame_dt) / self.rtf for s in sim.sensors}
        if self._stale_due is None:
            self._stale_due = {name: now + period for name, period in periods.items()}
        ns = self._stale_clock.sec * 1_000_000_000 + self._stale_clock.nanosec + round(sim.frame_dt * 1.0e+9)
        self._stale_clock = Time(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000)
        self.clock_pub.publish(Clock(clock=self._stale_clock))
        for s in sim.sensors:
            msg = self._stale.get(s.name)
            if msg is None or now < self._stale_due[s.name]:
                continue
            msg.header.stamp = self._stale_clock
            self.publishers_[s.name].publish(msg)
            self._stale_due[s.name] = max(self._stale_due[s.name] + periods[s.name], now + 0.5 * periods[s.name])

    def _report_timing(self) -> dict[str, Any]:
        """
        """
        out: dict[str, Any] = {"frames": len(self._timing), "rtf_target": self.rtf}

        if len(self._timing) > 1:
            (w0, s0), (w1, s1) = self._timing[0], self._timing[-1]
            out["rtf"] = (s1 - s0) / max(w1 - w0, 1.0e-9)

        if self._step_ms:
            steps = np.asarray(self._step_ms)
            budget = 1000.0 * self.sim.frame_dt / self.rtf if self.sim else 0.0
            out["step_ms_p50"] = float(np.percentile(steps, 50))
            out["step_ms_p99"] = float(np.percentile(steps, 99))
            out["step_ms_max"] = float(steps.max())
            out["budget_ms"] = budget
            out["overruns"] = int((steps > budget).sum())

        self._timing.clear()
        self._step_ms.clear()

        return out

    def _set_rtf(self, rtf: float) -> None:
        """
        Sets the real time factor and timer period.
        """
        if self.sim is None or rtf <= 0.0 or abs(rtf - self.rtf) < 1.0e-9:
            return

        self.rtf = rtf
        self._rtf_window = None

        if self.timer is not None:
            self.destroy_timer(self.timer)
        self.timer = self.create_timer(timer_period_sec=self.sim.frame_dt / self.rtf, callback=self._on_timer, clock=RosClock(clock_type=ClockType.STEADY_TIME))

    def destroy_node(self) -> None:
        """
        Tears down the simulator before the node.
        """
        self._teardown()
        super().destroy_node()


def main(args=None) -> None:
    """
    Runs the node until shutdown.
    """
    rclpy.init(args=args)
    node = None
    try:
        node = NewtonianNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        # launch forwards a second SIGINT, ignore it during cleanup
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
