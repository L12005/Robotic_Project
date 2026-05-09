from __future__ import annotations

import subprocess
from typing import Optional

import rclpy
from hmi_interfaces.msg import BehaviorState
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

try:
    from gz.msgs10.entity_pb2 import Entity
    from gz.msgs10.material_color_pb2 import MaterialColor
    from gz.transport13 import Node as GzNode
except ImportError:  # pragma: no cover - exercised only on non-Gazebo systems
    Entity = None
    GzNode = None
    MaterialColor = None

from hmi_feedback.led_strip_patterns import LedFrame, build_led_frame


class LedStripController(Node):
    def __init__(self) -> None:
        super().__init__('led_strip_controller')

        self._behavior_state_topic = self.declare_parameter(
            'behavior_state_topic',
            '/hmi/control/behavior_state',
            ParameterDescriptor(description='BehaviorState topic that drives the feedback LED strip.'),
        ).value
        self._world_name = self.declare_parameter(
            'world_name',
            'elevator_yield_human_control',
            ParameterDescriptor(description='Gazebo world that owns the visual_config service.'),
        ).value
        self._model_name = self.declare_parameter(
            'model_name',
            'starship_delivery_robot_model',
            ParameterDescriptor(description='Robot model that contains the LED segment visuals.'),
        ).value
        self._link_name = self.declare_parameter(
            'link_name',
            'base_link',
            ParameterDescriptor(description='Robot link that contains the LED segment visuals.'),
        ).value
        self._segment_count = int(self.declare_parameter('segment_count', 24).value)
        self._animation_rate_hz = float(self.declare_parameter('animation_rate_hz', 8.0).value)
        self._flow_speed_segments_per_sec = float(self.declare_parameter('flow_speed_segments_per_sec', 8.0).value)
        self._hard_stop_fast_blink_hz = float(self.declare_parameter('hard_stop_fast_blink_hz', 3.0).value)
        self._resume_duration = float(self.declare_parameter('resume_duration', 1.2).value)
        self._enable_runtime_material_updates = bool(
            self.declare_parameter('enable_runtime_material_updates', True).value
        )

        self._latest_msg: BehaviorState | None = None
        self._last_resuming = False
        self._resume_started_sec: float | None = None
        self._last_internal_state = ''
        self._last_frame_signature: tuple[str, str] | None = None
        self._last_segment_keys: list[tuple[float, float, float, float] | None] = [None] * self._segment_count
        self._material_retry_after_sec = 0.0
        self._warned_gz_failure = False
        self._gz_publisher = None
        if GzNode is not None and MaterialColor is not None:
            self._gz_publisher = GzNode().advertise(
                f'/world/{self._world_name}/material_color',
                MaterialColor,
            )

        self.create_subscription(BehaviorState, self._behavior_state_topic, self._on_behavior_state, 20)
        self.create_timer(1.0 / max(self._animation_rate_hz, 1e-6), self._tick)

        self.get_logger().info(
            'LED strip controller ready. '
            f'world={self._world_name} model={self._model_name} segments={self._segment_count} '
            f'behavior_topic={self._behavior_state_topic}'
        )

    def _on_behavior_state(self, msg: BehaviorState) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if msg.is_resuming and not self._last_resuming:
            self._resume_started_sec = now_sec
        elif not msg.is_resuming:
            self._resume_started_sec = None
        self._last_resuming = bool(msg.is_resuming)

        if msg.avoidance_started_event:
            self.get_logger().info('feedback_sound_cue yielding_start_soft_chime')
        if msg.internal_state == 'HardStop' and self._last_internal_state != 'HardStop':
            self.get_logger().warning('feedback_sound_cue hard_stop_warning')

        self._last_internal_state = msg.internal_state
        self._latest_msg = msg

    def _tick(self) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        msg = self._latest_msg
        if msg is None:
            frame = build_led_frame(
                internal_state='Idle',
                motion_direction='none',
                is_resuming=False,
                now_sec=now_sec,
                segment_count=self._segment_count,
                flow_speed_segments_per_sec=self._flow_speed_segments_per_sec,
                hard_stop_fast_blink_hz=self._hard_stop_fast_blink_hz,
                resume_duration=self._resume_duration,
                resume_start_sec=None,
            )
        else:
            frame = build_led_frame(
                internal_state=msg.internal_state,
                motion_direction=msg.motion_direction,
                is_resuming=bool(msg.is_resuming),
                now_sec=now_sec,
                segment_count=self._segment_count,
                flow_speed_segments_per_sec=self._flow_speed_segments_per_sec,
                hard_stop_fast_blink_hz=self._hard_stop_fast_blink_hz,
                resume_duration=self._resume_duration,
                resume_start_sec=self._resume_started_sec,
            )

        if frame is None:
            return
        self._maybe_log_frame(frame)
        if self._enable_runtime_material_updates and now_sec >= self._material_retry_after_sec:
            self._publish_frame_to_gazebo(frame, now_sec)

    def _maybe_log_frame(self, frame: LedFrame) -> None:
        signature = (frame.mode, frame.direction)
        if signature == self._last_frame_signature:
            return
        self.get_logger().info(f'led_strip_frame mode={frame.mode} direction={frame.direction}')
        self._last_frame_signature = signature

    def _publish_frame_to_gazebo(self, frame: LedFrame, now_sec: float) -> None:
        for index, segment in enumerate(frame.segments):
            key = (
                round(segment.red, 3),
                round(segment.green, 3),
                round(segment.blue, 3),
                round(segment.intensity, 3),
            )
            if self._last_segment_keys[index] == key:
                continue
            if self._set_segment_material(index, segment.red, segment.green, segment.blue, segment.intensity):
                self._last_segment_keys[index] = key
            else:
                self._material_retry_after_sec = now_sec + 1.0
                return

    def _set_segment_material(self, index: int, red: float, green: float, blue: float, intensity: float) -> bool:
        topic = f'/world/{self._world_name}/material_color'
        visual_name = f'led_segment_{index:02d}'
        diffuse_scale = 0.35 + 0.45 * intensity
        if self._gz_publisher is not None and MaterialColor is not None and Entity is not None:
            message = MaterialColor()
            message.entity.name = visual_name
            message.entity.type = Entity.VISUAL
            message.ambient.r = red * 0.10
            message.ambient.g = green * 0.10
            message.ambient.b = blue * 0.10
            message.ambient.a = 1.0
            message.diffuse.r = red * diffuse_scale
            message.diffuse.g = green * diffuse_scale
            message.diffuse.b = blue * diffuse_scale
            message.diffuse.a = 1.0
            message.emissive.r = red * intensity
            message.emissive.g = green * intensity
            message.emissive.b = blue * intensity
            message.emissive.a = 1.0
            message.entity_match = MaterialColor.FIRST
            if self._gz_publisher.publish(message):
                return True
            self._warn_once(f'Gazebo material_color publish failed for {visual_name}; will retry.')
            return False

        request = (
            f'entity {{ name: "{visual_name}" type: VISUAL }} '
            f'ambient {{ r: {red * 0.10:.4f} g: {green * 0.10:.4f} b: {blue * 0.10:.4f} a: 1.0 }} '
            f'diffuse {{ r: {red * diffuse_scale:.4f} g: {green * diffuse_scale:.4f} b: {blue * diffuse_scale:.4f} a: 1.0 }} '
            f'emissive {{ r: {red * intensity:.4f} g: {green * intensity:.4f} b: {blue * intensity:.4f} a: 1.0 }} '
            'entity_match: FIRST'
        )
        command = [
            'gz',
            'topic',
            '-t',
            topic,
            '-m',
            'gz.msgs.MaterialColor',
            '-p',
            request,
        ]
        try:
            result = subprocess.run(command, capture_output=True, text=True, check=False)
        except FileNotFoundError:
            self._warn_once('Could not find `gz`; LED strip material updates are disabled until Gazebo is available.')
            return False

        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            self._warn_once(f'Gazebo visual_config update failed for {visual_name}: {detail}')
            return False
        return True

    def _warn_once(self, message: str) -> None:
        if self._warned_gz_failure:
            return
        self.get_logger().warning(message)
        self._warned_gz_failure = True


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LedStripController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
