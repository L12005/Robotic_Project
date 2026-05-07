import json
import socket
import sys
import time

import pygame


SERVER_HOST = '127.0.0.1'
SERVER_PORT = 8765
PUBLISH_RATE_HZ = 30.0
LEFT_X_AXIS = 0
LEFT_Y_AXIS = 1
A_BUTTON_INDEX = 0
INVERT_LEFT_Y = True


def main() -> int:
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() <= 0:
        print('No joystick detected by pygame.')
        return 1

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f'Using joystick: {joystick.get_name()}')
    print(f'Connecting to WSL receiver at {SERVER_HOST}:{SERVER_PORT} ...')

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((SERVER_HOST, SERVER_PORT))
    print('Connected. Move the left stick to walk, press A to stop.')

    period = 1.0 / max(PUBLISH_RATE_HZ, 1.0)
    try:
        while True:
            pygame.event.pump()

            left_x = float(joystick.get_axis(LEFT_X_AXIS))
            left_y = float(joystick.get_axis(LEFT_Y_AXIS))
            if INVERT_LEFT_Y:
                left_y = -left_y
            a_pressed = bool(joystick.get_button(A_BUTTON_INDEX))

            payload = {
                'left_x': left_x,
                'left_y': left_y,
                'a_pressed': a_pressed,
            }
            sock.sendall((json.dumps(payload) + '\n').encode('utf-8'))
            time.sleep(period)
    except KeyboardInterrupt:
        print('\nStopped by user.')
        return 0
    finally:
        try:
            sock.close()
        except OSError:
            pass
        pygame.joystick.quit()
        pygame.quit()


if __name__ == '__main__':
    sys.exit(main())
