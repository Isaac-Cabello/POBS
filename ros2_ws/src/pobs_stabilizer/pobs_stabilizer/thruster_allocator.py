#!/usr/bin/env python3

import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float64


def numbers(text, count, label):
    values = np.asarray(
        [float(value) for value in text.split()],
        dtype=float,
    )
    if values.shape != (count,) or not np.all(np.isfinite(values)):
        raise ValueError(f"{label}: expected {count} finite values")
    return values


def pose_matrix(element):
    """Read a plain xyz/rpy pose; reject unsupported representations."""
    pose = element.find("pose")
    transform = np.eye(4)

    if pose is None:
        return transform

    if pose.get("relative_to"):
        raise ValueError(
            "This parser does not support pose relative_to. "
            "Use a libsdformat-based resolver for that model."
        )

    if pose.get("rotation_format", "euler_rpy") != "euler_rpy":
        raise ValueError("Only euler_rpy poses are supported")

    values = numbers(pose.text or "", 6, "pose")
    degrees = pose.get("degrees", "false").lower()
    if degrees not in ("false", "true", "0", "1"):
        raise ValueError("Invalid pose degrees attribute")

    transform[:3, :3] = Rotation.from_euler(
        "xyz",
        values[3:],
        degrees=degrees in ("true", "1"),
    ).as_matrix()
    transform[:3, 3] = values[:3]
    return transform


def boolean(text):
    value = text.strip().lower()
    if value in ("true", "1"):
        return True
    if value in ("false", "0"):
        return False
    raise ValueError(f"Invalid boolean: {text!r}")


def discover_thrusters(path, body_frame, force_cap):
    """Extract geometry from the direct-link POBS model structure."""
    root = ET.parse(path).getroot()
    models = [root] if root.tag == "model" else root.findall("model")

    if len(models) != 1:
        raise ValueError(
            "sdf_path must point to one model SDF, not a world file"
        )

    model = models[0]
    if model.find("include") is not None or model.find("model") is not None:
        raise ValueError("Includes and nested models are not supported")
    if model.find("frame") is not None:
        raise ValueError("Explicit SDF frames require a frame-aware parser")

    link_elements = model.findall("link")
    joint_elements = model.findall("joint")
    links = {link.get("name"): link for link in link_elements}
    joints = {joint.get("name"): joint for joint in joint_elements}

    if (
        None in links
        or len(links) != len(link_elements)
        or None in joints
        or len(joints) != len(joint_elements)
    ):
        raise ValueError("Missing or duplicate link/joint names")
    if body_frame not in links:
        raise ValueError(f"Body link {body_frame!r} not found")

    # Plain link poses are relative to the model frame.
    body_from_model = np.linalg.inv(pose_matrix(links[body_frame]))
    thrusters = []
    used_joints = set()
    used_topics = set()

    for plugin in model.findall("plugin"):
        if plugin.get("name") != "gz::sim::systems::Thruster":
            continue

        if boolean(plugin.findtext("use_angvel_cmd", "false")):
            raise ValueError(
                "Angular-velocity command mode is not supported; "
                "use force-command mode."
            )

        joint_name = plugin.findtext("joint_name", "").strip()
        if joint_name not in joints or joint_name in used_joints:
            raise ValueError(f"Missing or duplicate thruster joint: {joint_name}")
        used_joints.add(joint_name)

        joint = joints[joint_name]
        child = joint.findtext("child", "").strip()
        parent = joint.findtext("parent", "").strip()

        if parent != body_frame:
            raise ValueError(
                f"{joint_name}: expected direct parent {body_frame!r}; "
                "articulated/indirect mounts are not supported"
            )
        if child not in links:
            raise ValueError(f"{joint_name}: child link not found")
        if joint.get("type") != "revolute":
            raise ValueError(f"{joint_name}: expected a revolute joint")

        # Current POBS joint origins coincide with child-link origins.
        # Reject offsets rather than assuming where the plugin applies force.
        joint_pose = pose_matrix(joint)
        if not np.allclose(joint_pose, np.eye(4), atol=1e-10):
            raise ValueError(
                f"{joint_name}: nonidentity joint poses are not supported"
            )

        xyz = joint.find("axis/xyz")
        if xyz is None or xyz.get("expressed_in"):
            raise ValueError(
                f"{joint_name}: expected an unqualified axis/xyz"
            )
        if boolean(joint.findtext(
            "axis/use_parent_model_frame", "false"
        )):
            raise ValueError("Legacy parent-model-frame axes unsupported")

        axis = numbers(xyz.text or "", 3, f"{joint_name} axis")
        norm = np.linalg.norm(axis)
        if norm < 1e-9:
            raise ValueError(f"{joint_name}: zero axis")

        body_from_child = (
            body_from_model @ pose_matrix(links[child])
        )
        position = body_from_child[:3, 3]
        direction = body_from_child[:3, :3] @ (axis / norm)

        # These are plugin input-force limits, not joint effort limits.
        lower = float(plugin.findtext("min_thrust_cmd", "-1000"))
        upper = float(plugin.findtext("max_thrust_cmd", "1000"))
        if (
            not np.isfinite(lower)
            or not np.isfinite(upper)
            or lower > 0
            or upper < 0
            or lower > upper
        ):
            raise ValueError(f"{joint_name}: invalid thrust limits")

        if force_cap > 0:
            lower = max(lower, -force_cap)
            upper = min(upper, force_cap)

        namespace = plugin.findtext(
            "namespace", model.get("name", "")
        ).strip("/")
        if not namespace:
            raise ValueError("A model name or plugin namespace is required")

        custom_topic = plugin.findtext("topic", "").strip()
        if custom_topic:
            raise ValueError(
                f"{joint_name}: custom plugin topics are not supported "
                "by this POBS-specific version"
            )

        topic = f"/model/{namespace}/joint/{joint_name}/cmd_thrust"
        if topic in used_topics:
            raise ValueError(f"Duplicate thruster command topic: {topic}")
        used_topics.add(topic)

        thrusters.append({
            "name": child,
            "joint": joint_name,
            "position": position,
            "direction": direction,
            "lower": lower,
            "upper": upper,
            "topic": topic,
            "default_limits": (
                plugin.find("min_thrust_cmd") is None
                or plugin.find("max_thrust_cmd") is None
            ),
        })

    if not thrusters:
        raise ValueError("No Gazebo Thruster plugins found")

    return thrusters


class ThrusterAllocator(Node):
    def __init__(self):
        super().__init__("thruster_allocator")

        defaults = {
            "sdf_path": "",
            "body_frame": "base_link",
            "publish_rate": 50.0,
            "command_timeout": 0.25,
            "regularization": 1e-6,
            "wrench_weights": [1.0] * 6,
            # 0 means use the SDF/plugin limits without an extra cap.
            "force_cap": 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(
                name, value, ParameterDescriptor(read_only=True)
            )

        def param(name):
            return self.get_parameter(name).value

        self.body_frame = str(param("body_frame"))
        rate = float(param("publish_rate"))
        self.timeout = float(param("command_timeout"))
        self.regularization = float(param("regularization"))
        force_cap = float(param("force_cap"))

        values = [rate, self.timeout, self.regularization, force_cap]
        if (
            not np.all(np.isfinite(values))
            or rate <= 0
            or self.timeout <= 0
            or self.regularization < 0
            or force_cap < 0
        ):
            raise ValueError("Invalid rate, timeout, regularization, or cap")

        self.weights = np.asarray(param("wrench_weights"), dtype=float)
        if (
            self.weights.shape != (6,)
            or not np.all(np.isfinite(self.weights))
            or np.any(self.weights <= 0)
        ):
            raise ValueError("wrench_weights needs six positive finite values")

        path = Path(str(param("sdf_path"))).expanduser()
        if not path.is_file():
            raise ValueError(f"sdf_path is not a file: {path}")

        self.thrusters = discover_thrusters(
            path, self.body_frame, force_cap
        )
        self.count = len(self.thrusters)

        self.B = np.column_stack([
            np.concatenate((
                t["direction"],
                np.cross(t["position"], t["direction"]),
            ))
            for t in self.thrusters
        ])
        self.lower = np.array([t["lower"] for t in self.thrusters])
        self.upper = np.array([t["upper"] for t in self.thrusters])
        self.active = np.flatnonzero(self.upper > self.lower)

        self.A = self.weights[:, None] * self.B[:, self.active]
        if self.regularization:
            self.A = np.vstack((
                self.A,
                np.sqrt(self.regularization) * np.eye(len(self.active)),
            ))

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.thruster_publishers = [
            self.create_publisher(Float64, t["topic"], qos)
            for t in self.thrusters
        ]
        self.achieved_pub = self.create_publisher(
            WrenchStamped, "achieved_wrench", qos
        )
        self.residual_pub = self.create_publisher(
            WrenchStamped, "wrench_residual", qos
        )
        self.subscription = self.create_subscription(
            WrenchStamped, "cmd_wrench", self.on_wrench, qos
        )

        self.target = None
        self.received_at = None
        self.last_error = -float("inf")
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            1.0 / rate, self.tick, clock=self.steady_clock
        )

        active_B = self.B[:, self.active]
        rank = int(np.linalg.matrix_rank(active_B)) if len(self.active) else 0
        self.get_logger().info(f"Read {path.resolve()}")
        self.get_logger().info(
            f"Discovered {self.count} thrusters; active rank={rank}/6"
        )
        self.get_logger().info(f"Allocation matrix:\n{self.B}")

        for t in self.thrusters:
            self.get_logger().info(
                f"{t['name']}: position={t['position'].tolist()}, "
                f"direction={t['direction'].tolist()}, "
                f"limits=[{t['lower']}, {t['upper']}] N, "
                f"topic={t['topic']}"
            )
        if any(t["default_limits"] for t in self.thrusters):
            self.get_logger().warning(
                "Some limits use Gazebo defaults, not physical ratings. "
                "Set explicit SDF limits or supply force_cap."
            )
        if rank < 6:
            self.get_logger().warning(
                "Not all six wrench components are independently controllable."
            )

    def fail(self, reason):
        self.target = None
        self.received_at = None
        now = time.monotonic()
        if now - self.last_error >= 1.0:
            self.get_logger().error(reason)
            self.last_error = now
        self.publish(np.zeros(self.count), np.zeros(6))

    def on_wrench(self, msg):
        if msg.header.frame_id != self.body_frame:
            self.fail(
                f"Expected frame {self.body_frame!r}, "
                f"received {msg.header.frame_id!r}"
            )
            return

        w = msg.wrench
        target = np.array([
            w.force.x, w.force.y, w.force.z,
            w.torque.x, w.torque.y, w.torque.z,
        ])
        if not np.all(np.isfinite(target)):
            self.fail("Non-finite wrench rejected")
            return

        self.target = target
        self.received_at = time.monotonic()

    def tick(self):
        if (
            self.target is None
            or self.received_at is None
            or time.monotonic() - self.received_at >= self.timeout
        ):
            self.target = None
            self.received_at = None
            self.publish(np.zeros(self.count), np.zeros(6))
            return

        target = self.target.copy()
        forces = np.zeros(self.count)

        try:
            if len(self.active):
                b = self.weights * target
                if self.regularization:
                    b = np.concatenate((b, np.zeros(len(self.active))))

                solution = lsq_linear(
                    self.A,
                    b,
                    bounds=(
                        self.lower[self.active],
                        self.upper[self.active],
                    ),
                    tol=1e-8,
                    max_iter=100,
                )
                if (
                    not solution.success
                    or not np.all(np.isfinite(solution.x))
                ):
                    raise RuntimeError(str(solution.message))
                forces[self.active] = solution.x

            self.publish(forces, target)
        except Exception as exc:
            self.fail(f"Allocation failed: {exc}")

    def wrench_message(self, vector, stamp):
        msg = WrenchStamped()
        msg.header.frame_id = self.body_frame
        msg.header.stamp = stamp
        msg.wrench.force.x = float(vector[0])
        msg.wrench.force.y = float(vector[1])
        msg.wrench.force.z = float(vector[2])
        msg.wrench.torque.x = float(vector[3])
        msg.wrench.torque.y = float(vector[4])
        msg.wrench.torque.z = float(vector[5])
        return msg

    def publish(self, forces, target):
        for publisher, force in zip(self.thruster_publishers, forces):
            publisher.publish(Float64(data=float(force)))

        achieved = self.B @ forces
        stamp = self.get_clock().now().to_msg()
        self.achieved_pub.publish(self.wrench_message(achieved, stamp))
        self.residual_pub.publish(
            self.wrench_message(target - achieved, stamp)
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ThrusterAllocator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()