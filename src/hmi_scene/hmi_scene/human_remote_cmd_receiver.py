from __future__ import annotations

import json
import math
import socket
import threading

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


class HumanRemoteCmdReceiver(Node):
    def __init__(self) -> None:
        super().__init__('human_remote_cmd_receiver')

        cmd_vel_topic = str(
            self.declare_parameter(
                'cmd_vel_topic',
                '/hmi/human/cmd_vel',
                ParameterDescriptor(description='Human velocity topic consumed by the scene-side human controller.'),
            ).value
        )
        self._listen_host = str(
            self.declare_parameter(
                'listen_host',
                '0.0.0.0',
                ParameterDescriptor(description='TCP host interface used to receive forwarded controller data.'),
            ).value
        )
        self._listen_port = int(
            self.declare_parameter(
                'listen_port',
                8765,
                ParameterDescriptor(description='TCP port used to receive forwarded controller data.'),
            ).value
        )
        self._publish_rate_hz = float(
            self.declare_parameter(
                'publish_rate_hz',
                30.0,
                ParameterDescriptor(description='How often to republish the latest command to the human controller.'),
            ).value
        )
        self._fixed_speed = float(
            self.declare_parameter(
                'fixed_speed',
                1.0,
                ParameterDescriptor(description='Fixed walking speed used whenever the remote left stick leaves the deadzone.'),
            ).value
        )
        self._deadzone = float(
            self.declare_parameter(
                'deadzone',
                0.25,
                ParameterDescriptor(description='Minimum stick magnitude required to trigger walking.'),
            ).value
        )
        self._input_rotation_rad = float(
            self.declare_parameter(
                'input_rotation_rad',
                0.0,
                ParameterDescriptor(
                    description='Optional rotation applied to the incoming stick direction before mapping to world x/y.'
                ),
            ).value
        )

        self._publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_timer(1.0 / max(self._publish_rate_hz, 1.0), self._publish_latest_command)

        self._left_x = 0.0
        self._left_y = 0.0
        self._stop_pressed = False
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._server_thread = threading.Thread(target=self._run_server, daemon=True)
        self._server_thread.start()
        self._last_command_signature: tuple[float, float] | None = None

        self.get_logger().info(
            'Human remote cmd receiver is ready. '
            f'Listening on {self._listen_host}:{self._listen_port}, publishing to {cmd_vel_topic}.'
        )

    def _run_server(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self._listen_host, self._listen_port))
        server.listen(1)
        server.settimeout(1.0)

        try:
            while not self._stop_event.is_set():
                try:
                    conn, addr = server.accept()
                except socket.timeout:
                    continue
                self.get_logger().info(f'Windows controller sender connected from {addr[0]}:{addr[1]}.')
                conn.settimeout(1.0)
                buffer = ''
                try:
                    while not self._stop_event.is_set():
                        try:
                            chunk = conn.recv(4096)
                        except socket.timeout:
                            continue
                        if not chunk:
                            break
                        buffer += chunk.decode('utf-8', errors='ignore')
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            self._handle_line(line.strip())
                finally:
                    conn.close()
                    self.get_logger().info('Windows controller sender disconnected.')
                    with self._lock:
                        self._left_x = 0.0
                        self._left_y = 0.0
                        self._stop_pressed = False
        finally:
            server.close()

    def _handle_line(self, line: str) -> None:
        if not line:
            return
        try:
            payload = json.loads(line)
        except json.JSONDecodeError:
            return

        left_x = float(payload.get('left_x', 0.0))
        left_y = float(payload.get('left_y', 0.0))
        stop_pressed = bool(payload.get('a_pressed', False))

        with self._lock:
            self._left_x = left_x
            self._left_y = left_y
            self._stop_pressed = stop_pressed

    def _publish_latest_command(self) -> None:
        with self._lock:
            left_x = self._left_x
            left_y = self._left_y
            stop_pressed = self._stop_pressed

        twist = Twist()
        rotated_x = left_x
        rotated_y = left_y
        if abs(self._input_rotation_rad) > 1e-9:
            cos_yaw = math.cos(self._input_rotation_rad)
            sin_yaw = math.sin(self._input_rotation_rad)
            rotated_x = left_x * cos_yaw - left_y * sin_yaw
            rotated_y = left_x * sin_yaw + left_y * cos_yaw

        magnitude = math.hypot(rotated_x, rotated_y)
        if not stop_pressed and magnitude >= self._deadzone:
            direction_x = rotated_x / max(magnitude, 1e-6)
            direction_y = rotated_y / max(magnitude, 1e-6)
            twist.linear.x = direction_x * self._fixed_speed
            twist.linear.y = direction_y * self._fixed_speed

        self._publisher.publish(twist)
        self._maybe_log_command(twist, stop_pressed)

    def _maybe_log_command(self, twist: Twist, stop_pressed: bool) -> None:
        signature = (round(float(twist.linear.x), 3), round(float(twist.linear.y), 3))
        if signature == self._last_command_signature:
            return
        state = 'stopped' if stop_pressed or signature == (0.0, 0.0) else 'moving'
        self.get_logger().info(
            'human_remote_cmd '
            f'state={state} vx={twist.linear.x:.3f} vy={twist.linear.y:.3f}'
        )
        self._last_command_signature = signature

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._server_thread.is_alive():
            self._server_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HumanRemoteCmdReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
