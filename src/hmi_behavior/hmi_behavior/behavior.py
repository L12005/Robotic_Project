from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from hmi_interfaces.msg import ActorState, BehaviorState, ObstacleState
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

from hmi_behavior.robot_state import AggregatedState, ActorSnapshot, GoalSnapshot, ObstacleSnapshot
from hmi_behavior.state_machine import BehaviorStateMachine, ControllerConfig


class BehaviorNode(Node):
    def __init__(self) -> None:
        super().__init__('hmi_behavior')
        self._world_state = AggregatedState()
        self._config = self._declare_and_load_config()
        self._state_machine = BehaviorStateMachine(self._config)
        self._last_debug_log_sec = 0.0
        self._last_debug_signature: tuple[str, str, float, float] | None = None

        robot_state_topic = self.declare_parameter(
            'robot_state_topic',
            '/hmi/scene/robot_state',
            ParameterDescriptor(description='Input topic for robot ActorState.'),
        ).value
        human_state_topic = self.declare_parameter(
            'human_state_topic',
            '/hmi/scene/human_state',
            ParameterDescriptor(description='Input topic for human ActorState.'),
        ).value
        obstacle_state_topic = self.declare_parameter(
            'obstacle_state_topic',
            '/hmi/scene/obstacle_state',
            ParameterDescriptor(description='Input topic for obstacle overlays.'),
        ).value
        static_map_topic = self.declare_parameter(
            'static_map_topic',
            '/hmi/scene/static_map',
            ParameterDescriptor(description='Primary static OccupancyGrid topic for A* planning.'),
        ).value
        map_state_topic = self.declare_parameter(
            'map_state_topic',
            '/hmi/scene/map_state',
            ParameterDescriptor(description='Fallback OccupancyGrid topic if static_map is unavailable.'),
        ).value
        goal_topic = self.declare_parameter(
            'goal_topic',
            '/hmi/control/goal_pose',
            ParameterDescriptor(
                description='Goal input interface. Use geometry_msgs/PoseStamped to inject target points.'
            ),
        ).value
        cmd_vel_topic = self.declare_parameter(
            'cmd_vel_topic',
            '/hmi/control/cmd_vel',
            ParameterDescriptor(description='Output velocity command topic.'),
        ).value
        behavior_state_topic = self.declare_parameter(
            'behavior_state_topic',
            '/hmi/control/behavior_state',
            ParameterDescriptor(description='Output HMI behavior state topic.'),
        ).value

        control_rate_hz = self.declare_parameter(
            'control_rate_hz',
            10.0,
            ParameterDescriptor(description='Main control loop frequency in Hz.'),
        ).value

        self.create_subscription(ActorState, robot_state_topic, self._on_robot_state, 10)
        self.create_subscription(ActorState, human_state_topic, self._on_human_state, 10)
        self.create_subscription(ObstacleState, obstacle_state_topic, self._on_obstacle_state, 50)
        self.create_subscription(OccupancyGrid, static_map_topic, self._on_static_map, 10)
        self.create_subscription(OccupancyGrid, map_state_topic, self._on_map_state, 10)
        self.create_subscription(PoseStamped, goal_topic, self._on_goal_pose, 10)

        self._cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._behavior_state_publisher = self.create_publisher(BehaviorState, behavior_state_topic, 10)
        self.create_timer(1.0 / control_rate_hz, self._tick)

        self.get_logger().info(
            'hmi_behavior started. Waiting for scene inputs on '
            f'{robot_state_topic}, {human_state_topic}, {obstacle_state_topic}, '
            f'{static_map_topic} (primary) / {map_state_topic} (fallback), and future goal poses on {goal_topic}.'
        )

    def _declare_and_load_config(self) -> ControllerConfig:
        return ControllerConfig(
            goal_tolerance=self.declare_parameter('goal_tolerance', 0.20).value,
            max_forward_speed=self.declare_parameter('max_forward_speed', 0.45).value,
            max_reverse_speed=self.declare_parameter('max_reverse_speed', 0.25).value,
            max_angular_speed=self.declare_parameter('max_angular_speed', 1.20).value,
            angular_gain=self.declare_parameter('angular_gain', 1.40).value,
            linear_gain=self.declare_parameter('linear_gain', 0.70).value,
            wait_duration=self.declare_parameter('wait_duration', 1.00).value,
            safety_radius=self.declare_parameter('safety_radius', 0.80).value,
            human_zone_time=self.declare_parameter('human_zone_time', 1.20).value,
            human_exit_time=self.declare_parameter('human_exit_time', 2.00).value,
            human_zone_half_width=self.declare_parameter('human_zone_half_width', 0.50).value,
            reverse_obstacle_distance=self.declare_parameter('reverse_obstacle_distance', 0.60).value,
            reverse_obstacle_half_width=self.declare_parameter('reverse_obstacle_half_width', 0.45).value,
            reverse_distance_elevator=self.declare_parameter('reverse_distance_elevator', 0.70).value,
            reverse_distance_open_area=self.declare_parameter('reverse_distance_open_area', 0.55).value,
            lateral_offset_elevator=self.declare_parameter('lateral_offset_elevator', 0.45).value,
            lateral_offset_open_area=self.declare_parameter('lateral_offset_open_area', 0.80).value,
            side_probe_distance=self.declare_parameter('side_probe_distance', 1.00).value,
            narrow_width_threshold=self.declare_parameter('narrow_width_threshold', 1.60).value,
            heading_slow_threshold=self.declare_parameter('heading_slow_threshold', 0.50).value,
            heading_stop_threshold=self.declare_parameter('heading_stop_threshold', 1.20).value,
            resume_duration=self.declare_parameter('resume_duration', 1.20).value,
            path_lookahead_distance=self.declare_parameter('path_lookahead_distance', 0.35).value,
            dynamic_obstacle_inflation=self.declare_parameter('dynamic_obstacle_inflation', 0.18).value,
            human_body_radius=self.declare_parameter('human_body_radius', 0.40).value,
            human_forward_min_depth=self.declare_parameter('human_forward_min_depth', 0.35).value,
            yield_sample_count=self.declare_parameter('yield_sample_count', 16).value,
            planner_occupancy_threshold=self.declare_parameter('planner_occupancy_threshold', 50).value,
        )

    def _on_robot_state(self, msg: ActorState) -> None:
        self._world_state.robot = ActorSnapshot.from_msg(msg)

    def _on_human_state(self, msg: ActorState) -> None:
        self._world_state.human = ActorSnapshot.from_msg(msg)

    def _on_obstacle_state(self, msg: ObstacleState) -> None:
        snapshot = ObstacleSnapshot.from_msg(msg)
        obstacle_id = snapshot.obstacle_id or f'obstacle_{len(self._world_state.obstacles)}'
        self._world_state.obstacles[obstacle_id] = snapshot

    def _on_static_map(self, msg: OccupancyGrid) -> None:
        self._world_state.static_map = msg

    def _on_map_state(self, msg: OccupancyGrid) -> None:
        self._world_state.map_state = msg

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        self._world_state.goal = GoalSnapshot.from_msg(msg)
        self.get_logger().info(
            'Received control goal '
            f'({self._world_state.goal.x:.2f}, {self._world_state.goal.y:.2f}) in frame '
            f'{self._world_state.goal.frame_id or "<empty>"}'
        )

    def _tick(self) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        output = self._state_machine.step(self._world_state, now_sec)

        twist_msg = Twist()
        twist_msg.linear.x = float(output.target_linear_x)
        twist_msg.angular.z = float(output.target_angular_z)
        self._cmd_vel_publisher.publish(twist_msg)

        behavior_msg = BehaviorState()
        behavior_msg.header.stamp = self.get_clock().now().to_msg()
        behavior_msg.current_state = output.behavior_state
        behavior_msg.internal_state = output.internal_state.value
        behavior_msg.reason = output.reason
        behavior_msg.target_linear_x = float(output.target_linear_x)
        behavior_msg.target_angular_z = float(output.target_angular_z)
        self._behavior_state_publisher.publish(behavior_msg)
        self._maybe_log_debug(output, now_sec)

    def _maybe_log_debug(self, output, now_sec: float) -> None:
        signature = (
            output.behavior_state,
            output.internal_state.value,
            output.reason,
            round(float(output.target_linear_x), 3),
            round(float(output.target_angular_z), 3),
        )
        if signature == self._last_debug_signature and now_sec - self._last_debug_log_sec < 1.0:
            return

        robot = self._world_state.robot
        goal = self._world_state.goal
        robot_text = 'robot=(missing)'
        if robot is not None:
            robot_text = f'robot=({robot.x:.2f}, {robot.y:.2f}, yaw={robot.yaw:.2f})'
        goal_text = 'goal=(missing)'
        if goal is not None:
            goal_text = f'goal=({goal.x:.2f}, {goal.y:.2f})'

        self.get_logger().info(
            'control_output '
            f'state={output.behavior_state} internal_state={output.internal_state.value} '
            f'reason={output.reason or "none"} '
            f'cmd=(linear_x={output.target_linear_x:.3f}, angular_z={output.target_angular_z:.3f}) '
            f'{robot_text} {goal_text}'
        )
        self._last_debug_log_sec = now_sec
        self._last_debug_signature = signature

    def destroy_node(self) -> bool:
        self._publish_zero_velocity()
        return super().destroy_node()

    def _publish_zero_velocity(self) -> None:
        twist_msg = Twist()
        self._cmd_vel_publisher.publish(twist_msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = BehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
