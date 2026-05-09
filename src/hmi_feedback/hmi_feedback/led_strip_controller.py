from __future__ import annotations

import subprocess
from typing import Optional

import rclpy
from hmi_interfaces.msg import BehaviorState
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

try:
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.scene_pb2 import Scene
    from gz.msgs10.visual_pb2 import Visual
    from gz.transport13 import Node as GzNode
except ImportError:  # pragma: no cover - exercised only on non-Gazebo systems
    Boolean = None
    Empty = None
    GzNode = None
    Scene = None
    Visual = None

from hmi_feedback.led_strip_patterns import LedFrame, build_led_frame, select_display_frame
from hmi_feedback.material_refresh import should_force_material_refresh


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
        self._startup_material_refresh_sec = float(
            self.declare_parameter('startup_material_refresh_sec', 3.0).value
        )
        self._startup_material_refresh_interval_sec = float(
            self.declare_parameter('startup_material_refresh_interval_sec', 0.5).value
        )
        self._steady_material_refresh_sec = float(
            self.declare_parameter('steady_material_refresh_sec', 1.0).value
        )
        self._enable_runtime_material_updates = bool(
            self.declare_parameter('enable_runtime_material_updates', True).value
        )

        self._latest_msg: BehaviorState | None = None
        self._last_resuming = False
        self._resume_started_sec: float | None = None
        self._last_internal_state = ''
        self._last_behavior_signature: tuple[str, str, bool] | None = None
        self._last_desired_frame: LedFrame | None = None
        self._last_frame_signature: tuple[str, str] | None = None
        self._last_segment_keys: list[tuple[float, float, float, float] | None] = [None] * self._segment_count
        self._material_retry_after_sec = 0.0
        self._force_material_refresh_until_sec = 0.0
        self._last_forced_material_refresh_sec = 0.0
        self._last_full_material_refresh_sec = 0.0
        self._initial_material_sync_complete = False
        self._warned_gz_failure = False
        self._warned_gz_waiting_for_visual_ids = False
        self._visual_ids: dict[str, int] = {}
        self._visual_ids_ready = False
        self._gz_node = None
        self._gz_publisher = None
        if GzNode is not None:
            self._gz_node = GzNode()

        self.create_subscription(BehaviorState, self._behavior_state_topic, self._on_behavior_state, 20)
        self.create_timer(1.0 / max(self._animation_rate_hz, 1e-6), self._tick)

        self.get_logger().info(
            'LED strip controller ready. '
            f'world={self._world_name} model={self._model_name} segments={self._segment_count} '
            f'behavior_topic={self._behavior_state_topic}'
        )

    def _on_behavior_state(self, msg: BehaviorState) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        signature = (msg.internal_state, msg.motion_direction, bool(msg.is_resuming))
        if signature != self._last_behavior_signature:
            self._force_material_refresh_until_sec = max(
                self._force_material_refresh_until_sec,
                now_sec + self._startup_material_refresh_sec,
            )
            self._last_behavior_signature = signature

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
            frame = select_display_frame(
                frame,
                self._last_desired_frame,
                now_sec=now_sec,
                segment_count=self._segment_count,
                flow_speed_segments_per_sec=self._flow_speed_segments_per_sec,
                hard_stop_fast_blink_hz=self._hard_stop_fast_blink_hz,
                resume_duration=self._resume_duration,
            )
        self._last_desired_frame = frame
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
        if not self._material_updates_ready(now_sec):
            self._material_retry_after_sec = now_sec + 0.25
            return

        force_refresh = should_force_material_refresh(
            now_sec=now_sec,
            initial_sync_complete=self._initial_material_sync_complete,
            force_refresh_until_sec=self._force_material_refresh_until_sec,
            last_full_refresh_sec=self._last_full_material_refresh_sec,
            steady_refresh_sec=self._steady_material_refresh_sec,
        )
        if (
            force_refresh
            and self._initial_material_sync_complete
            and now_sec - self._last_forced_material_refresh_sec < self._startup_material_refresh_interval_sec
            and all(key is not None for key in self._last_segment_keys)
        ):
            return

        for index, segment in enumerate(frame.segments):
            key = (
                round(segment.red, 3),
                round(segment.green, 3),
                round(segment.blue, 3),
                round(segment.intensity, 3),
            )
            if not force_refresh and self._last_segment_keys[index] == key:
                continue
            if self._set_segment_material(index, segment.red, segment.green, segment.blue, segment.intensity):
                self._last_segment_keys[index] = key
            else:
                self._material_retry_after_sec = now_sec + 1.0
                return
        if force_refresh:
            if not self._initial_material_sync_complete:
                self._initial_material_sync_complete = True
                self._force_material_refresh_until_sec = max(
                    self._force_material_refresh_until_sec,
                    now_sec + self._startup_material_refresh_sec,
                )
            self._last_forced_material_refresh_sec = now_sec
            self._last_full_material_refresh_sec = now_sec

    def _material_updates_ready(self, now_sec: float) -> bool:
        if self._gz_node is None or Empty is None or Scene is None:
            return True

        if self._visual_ids_ready:
            return True

        if self._refresh_visual_ids():
            self._last_segment_keys = [None] * self._segment_count
            self._initial_material_sync_complete = False
            self._last_forced_material_refresh_sec = 0.0
            self._last_full_material_refresh_sec = 0.0
            self._visual_ids_ready = True
            self.get_logger().info('Resolved Gazebo LED visual ids; refreshing LED strip materials.')
            return True

        if not self._warned_gz_waiting_for_visual_ids:
            self.get_logger().info('Waiting for Gazebo scene info before applying LED strip colors.')
            self._warned_gz_waiting_for_visual_ids = True
        return False

    def _refresh_visual_ids(self) -> bool:
        if self._gz_node is None or Empty is None or Scene is None:
            return False

        try:
            ok, scene = self._gz_node.request(
                f'/world/{self._world_name}/scene/info',
                Empty(),
                Empty,
                Scene,
                1000,
            )
        except RuntimeError as exc:
            self._warn_once(f'Could not query Gazebo scene info: {exc}')
            return False

        if not ok:
            return False

        visual_ids: dict[str, int] = {}
        for model in scene.model:
            if model.name != self._model_name:
                continue
            for link in model.link:
                if link.name != self._link_name:
                    continue
                for visual in link.visual:
                    if visual.name.startswith('led_segment_'):
                        visual_ids[visual.name] = int(visual.id)

        expected_names = {f'led_segment_{index:02d}' for index in range(self._segment_count)}
        if not expected_names.issubset(visual_ids.keys()):
            return False

        self._visual_ids = visual_ids
        return True

    def _set_segment_material(self, index: int, red: float, green: float, blue: float, intensity: float) -> bool:
        visual_name = f'led_segment_{index:02d}'
        diffuse_scale = 0.35 + 0.45 * intensity
        if self._gz_node is not None and Visual is not None and Boolean is not None:
            message = Visual()
            message.id = self._visual_ids.get(visual_name, 0)
            message.name = visual_name
            message.type = Visual.VISUAL
            message.material.ambient.r = red * 0.10
            message.material.ambient.g = green * 0.10
            message.material.ambient.b = blue * 0.10
            message.material.ambient.a = 1.0
            message.material.diffuse.r = red * diffuse_scale
            message.material.diffuse.g = green * diffuse_scale
            message.material.diffuse.b = blue * diffuse_scale
            message.material.diffuse.a = 1.0
            message.material.emissive.r = red * intensity
            message.material.emissive.g = green * intensity
            message.material.emissive.b = blue * intensity
            message.material.emissive.a = 1.0
            message.material.lighting = True
            try:
                ok, response = self._gz_node.request(
                    f'/world/{self._world_name}/visual_config',
                    message,
                    Visual,
                    Boolean,
                    1000,
                )
            except RuntimeError as exc:
                self._warn_once(f'Gazebo visual_config request failed for {visual_name}: {exc}')
                return False
            if ok and response.data:
                return True
            self._warn_once(f'Gazebo visual_config rejected material update for {visual_name}.')
            return False

        topic = f'/world/{self._world_name}/visual_config'
        visual_id = self._visual_ids.get(visual_name, 0)
        for request_name in (visual_name,):
            request = (
                f'id: {visual_id} name: "{request_name}" type: VISUAL '
                'material { '
                f'ambient {{ r: {red * 0.10:.4f} g: {green * 0.10:.4f} b: {blue * 0.10:.4f} a: 1.0 }} '
                f'diffuse {{ r: {red * diffuse_scale:.4f} g: {green * diffuse_scale:.4f} b: {blue * diffuse_scale:.4f} a: 1.0 }} '
                f'emissive {{ r: {red * intensity:.4f} g: {green * intensity:.4f} b: {blue * intensity:.4f} a: 1.0 }} '
                'lighting: true '
                '}'
            )
            command = [
                'gz',
                'service',
                '-s',
                topic,
                '--reqtype',
                'gz.msgs.Visual',
                '--reptype',
                'gz.msgs.Boolean',
                '--timeout',
                '1000',
                '--req',
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
