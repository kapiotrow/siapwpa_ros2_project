import time
import math
import serial
import mecanum_pb2
import cv2
# import pygame
from camera_libs_symlink.FrameProcessors import LineKalmanProcessor 
import matplotlib.pyplot as plt
from camera_libs_symlink.line_follower import LineFollower

# ====== KONFIGURACJA ======
ROBOT_PORT = "/dev/ttyUSB0"
BAUDRATE = 19200

CAMERA_INDEX = 4
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 20.0


# pygame.init()
# screen = pygame.display.set_mode((640, 480))
# clock = pygame.time.Clock()
# print()


line_follower = LineFollower()
print("line follower initialized")


SPEED_MMPS = 50           # mała prędkość
DIRECTION_RAD = 3 * math.pi / 2  # TYŁ
OMEGA_RAD = 0.0           # brak obrotu

INTERVAL = 0.001            # 100 ms
VIDEO_FILENAME = "test_backwards.mp4"
# ==========================

def send_robot_command(ser, speed_mmps, direction_rad, omega_rads):
    request = mecanum_pb2.ControlRequest()
    request.speed_mmps = int(speed_mmps)
    request.rad = float(direction_rad)
    request.omega = float(omega_rads)
    ser.write(request.SerializeToString())

def main():
    # ===== ROBOT =====
    try:
        ser = serial.Serial(ROBOT_PORT, BAUDRATE, timeout=1)
        time.sleep(2)
        print("Połączono z robotem")
    except serial.SerialException as e:
        print("Błąd portu szeregowego:", e)
        return

    # ===== KAMERA =====
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("Nie można otworzyć kamery")
        ser.close()
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(
        VIDEO_FILENAME,
        fourcc,
        FPS,
        (FRAME_WIDTH, FRAME_HEIGHT)
    )

    if not out.isOpened():
        print("Nie można otworzyć pliku wideo")
        cap.release()
        ser.close()
        return

    print("Kamera uruchomiona")
    print("jazdaaaaa")

    last_command_time = 0

    try:
        while True:
            current_time = time.time()

            # --- Odczyt klatki ---
            ret, frame = cap.read()
            if not ret:
                print("Błąd odczytu klatki z kamery")
                break

            speed, direction, _ = line_follower.update(frame)

            # --- Sterowanie robotem ---
            # if current_time - last_command_time >= INTERVAL:
            send_robot_command(
                ser,
                500*speed,
                (-3.14/2 * (direction)) + 3.14/2,
                0
            )
            last_command_time = current_time

            time.sleep(INTERVAL)

    except KeyboardInterrupt:
        print("\nCTRL+C - zatrzymywanie robota")

    finally:
        # STOP robota
        send_robot_command(ser, 0, 0, 0)

        out.release()
        cap.release()
        ser.close()

        print("Połączenia zamknięte")

if __name__ == "__main__":
    main()
