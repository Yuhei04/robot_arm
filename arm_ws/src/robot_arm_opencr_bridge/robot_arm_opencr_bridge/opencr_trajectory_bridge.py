#!/usr/bin/env python3
import math
import time
from typing import Dict, List, Optional

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from sensor_msgs.msg import JointState


class OpenCRTrajectoryBridge(Node):
    def __init__(self) -> None:
        super().__init__("opencr_trajectory_bridge")

        self.declare_parameter("controller_name", "RM_controller")
        self.declare_parameter("execute", False)
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("command_delay", 0.0)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("max_step_deg", 10.0)
        self.declare_parameter("joint_min_deg", [-90.0, -90.0, -90.0, -90.0, -90.0, -90.0])
        self.declare_parameter("joint_max_deg", [90.0, 90.0, 90.0, 90.0, 90.0, 90.0])
        self.declare_parameter(
            "motor_joint_names",
            [
                "base_yaw_joint",
                "shoulder_pitch_joint",
                "upper_arm_roll_joint",
                "elbow_pitch_joint",
                "elbow_roll_joint",
                "wrist_pitch_joint",
            ],
        )
        self.declare_parameter("motor_signs", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("motor_offsets_deg", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.controller_name = self.get_parameter("controller_name").value
        self.execute_enabled = self._as_bool(self.get_parameter("execute").value)
        self.port = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baud").value)
        self.command_delay = float(self.get_parameter("command_delay").value)
        self.max_step_deg = float(self.get_parameter("max_step_deg").value)
        self.joint_min_deg = self._float_list_param("joint_min_deg")
        self.joint_max_deg = self._float_list_param("joint_max_deg")
        self.motor_joint_names = [str(v) for v in self.get_parameter("motor_joint_names").value]
        self.motor_signs = self._float_list_param("motor_signs")
        self.motor_offsets_deg = self._float_list_param("motor_offsets_deg")

        self._validate_params()
        self.serial_port = None
        self.last_motor_degs = [0.0] * 6
        self.last_joint_positions: Dict[str, float] = {name: 0.0 for name in self.motor_joint_names}
        self.active_goal_cancelled = False

        action_name = f"/{self.controller_name}/follow_joint_trajectory"
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        publish_period = 1.0 / float(self.get_parameter("publish_rate").value)
        self.create_timer(publish_period, self.publish_joint_state)

        mode = "EXECUTE" if self.execute_enabled else "DRY_RUN"
        self.get_logger().warn(f"OpenCR bridge started in {mode} mode on action {action_name}")
        self.get_logger().info("OpenCR q order: ID1 ID2 ID3 ID4 ID5 ID6")
        self.get_logger().info("Motor joint mapping: " + ", ".join(self.motor_joint_names))
        self.get_logger().info("Motor signs: " + ", ".join(f"{v:+.1f}" for v in self.motor_signs))

    def _as_bool(self, value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _float_list_param(self, name: str) -> List[float]:
        return [float(v) for v in self.get_parameter(name).value]

    def _validate_params(self) -> None:
        arrays = [self.motor_joint_names, self.motor_signs, self.motor_offsets_deg, self.joint_min_deg, self.joint_max_deg]
        if any(len(values) != 6 for values in arrays):
            raise ValueError("motor_joint_names, motor_signs, motor_offsets_deg, joint_min_deg, and joint_max_deg must each have 6 values")

    def goal_callback(self, goal_request):
        if not goal_request.trajectory.points:
            self.get_logger().error("Rejected empty trajectory")
            return GoalResponse.REJECT
        missing = [name for name in self.motor_joint_names if name not in goal_request.trajectory.joint_names]
        if missing:
            self.get_logger().error("Rejected trajectory missing joints: " + ", ".join(missing))
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        self.active_goal_cancelled = True
        self.get_logger().warn("Trajectory cancel requested; stopping after current command")
        return CancelResponse.ACCEPT

    def publish_joint_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.last_joint_positions.keys())
        msg.position = [self.last_joint_positions[name] for name in msg.name]
        self.joint_state_pub.publish(msg)

    def open_serial(self):
        if self.serial_port is not None:
            return self.serial_port
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise RuntimeError("pyserial is required. Install with: sudo apt install python3-serial") from exc
        self.serial_port = serial.Serial(self.port, baudrate=self.baud, timeout=1.0)
        time.sleep(2.0)
        self.get_logger().info(f"Opened OpenCR serial port {self.port} @ {self.baud}")
        return self.serial_port

    def trajectory_point_to_motor_degs(self, joint_names: List[str], positions: List[float]) -> List[float]:
        by_name = dict(zip(joint_names, positions))
        motor_degs = []
        for index, joint_name in enumerate(self.motor_joint_names):
            joint_rad = float(by_name[joint_name])
            motor_deg = math.degrees(joint_rad) * self.motor_signs[index] + self.motor_offsets_deg[index]
            motor_degs.append(motor_deg)
        return motor_degs

    def validate_motor_degs(self, motor_degs: List[float], previous: Optional[List[float]]) -> Optional[str]:
        for index, value in enumerate(motor_degs):
            if value < self.joint_min_deg[index] or value > self.joint_max_deg[index]:
                return f"ID{index + 1} target {value:.2f} deg outside {self.joint_min_deg[index]:.1f}..{self.joint_max_deg[index]:.1f} deg"
        if previous is not None:
            for index, value in enumerate(motor_degs):
                step = abs(value - previous[index])
                if step > self.max_step_deg:
                    return f"ID{index + 1} step {step:.2f} deg exceeds max_step_deg {self.max_step_deg:.1f}"
        return None

    def send_q(self, motor_degs: List[float]) -> None:
        line = "q " + " ".join(f"{value:.3f}" for value in motor_degs)
        if self.execute_enabled:
            serial_port = self.open_serial()
            serial_port.write((line + "\n").encode("ascii"))
            serial_port.flush()
            if self.command_delay > 0.0:
                time.sleep(self.command_delay)
        self.get_logger().info(line)

    def execute_callback(self, goal_handle):
        self.active_goal_cancelled = False
        trajectory = goal_handle.request.trajectory
        result = FollowJointTrajectory.Result()
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = trajectory.joint_names

        previous_motor_degs = list(self.last_motor_degs)
        start_time = time.monotonic()
        last_time = 0.0

        self.get_logger().info(f"Executing trajectory with {len(trajectory.points)} points")
        for point_index, point in enumerate(trajectory.points):
            if self.active_goal_cancelled or goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "Cancelled"
                return result

            target_time = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9
            sleep_time = max(0.0, target_time - (time.monotonic() - start_time))
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            if target_time < last_time:
                self.get_logger().warn("Trajectory point times are not monotonic")
            last_time = target_time

            motor_degs = self.trajectory_point_to_motor_degs(trajectory.joint_names, point.positions)
            error = self.validate_motor_degs(motor_degs, previous_motor_degs)
            if error:
                self.get_logger().error("Rejected trajectory: " + error)
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = error
                return result

            self.send_q(motor_degs)
            previous_motor_degs = motor_degs
            self.last_motor_degs = motor_degs
            by_name = dict(zip(trajectory.joint_names, point.positions))
            for name in self.last_joint_positions:
                self.last_joint_positions[name] = float(by_name[name])

            feedback.desired = point
            feedback.actual = point
            feedback.error.positions = [0.0] * len(trajectory.joint_names)
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"Sent point {point_index + 1}/{len(trajectory.points)}")

        self.publish_joint_state()
        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "OK"
        return result

    def destroy_node(self) -> bool:
        if self.serial_port is not None:
            self.serial_port.close()
            self.serial_port = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OpenCRTrajectoryBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
