import time
import math
import serial
import mecanum_pb2
import cv2
import pygame
from camera_libs_symlink.FrameProcessors import LineKalmanProcessor 
import matplotlib.pyplot as plt

# ====== KONFIGURACJA ======
ROBOT_PORT = "/dev/ttyUSB1"
BAUDRATE = 19200

CAMERA_INDEX = 4
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 20.0


pygame.init()
screen = pygame.display.set_mode((640, 480))
clock = pygame.time.Clock()


line_follower = LineKalmanProcessor()


SPEED_MMPS = 50           # mała prędkość
DIRECTION_RAD = 3 * math.pi / 2  # TYŁ
OMEGA_RAD = 0.0           # brak obrotu

INTERVAL = 0.1            # 100 ms
VIDEO_FILENAME = "test_backwards.mp4"
# ==========================

def send_robot_command(ser, speed_mmps, direction_rad, omega_rads):
    request = mecanum_pb2.ControlRequest()
    request.speed_mmps = int(speed_mmps)
    request.rad = float(direction_rad)
    request.omega = float(omega_rads)
    ser.write(request.SerializeToString())

def main():
    # # ===== ROBOT =====
    # try:
    #     ser = serial.Serial(ROBOT_PORT, BAUDRATE, timeout=1)
    #     time.sleep(2)
    #     print("Połączono z robotem")
    # except serial.SerialException as e:
    #     print("Błąd portu szeregowego:", e)
    #     return

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
    print(f"Nagrywanie do pliku: {VIDEO_FILENAME}")
    print("Ruch testowy: POWOLI W TYŁ (CTRL+C aby zakończyć)")

    last_command_time = 0

    try:
        while True:
            current_time = time.time()

            # # --- Sterowanie robotem ---
            # if current_time - last_command_time >= INTERVAL:
            #     send_robot_command(
            #         ser,
            #         SPEED_MMPS,
            #         DIRECTION_RAD,
            #         OMEGA_RAD
            #     )
            #     last_command_time = current_time

            # --- Odczyt klatki ---
            ret, frame = cap.read()
            if not ret:
                print("Błąd odczytu klatki z kamery")
                break
            
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # out.write(frame)

            new_frame = line_follower.process(frame)
            new_frame  = frame.swapaxes(0, 1)

            surface = pygame.surfarray.make_surface(new_frame)
            new_frame[new_frame.shape[0]//2 + 1][new_frame.shape[1]//2 + 1] = [255, 0, 0] 
            # print(new_frame[new_frame.shape[0]//2][new_frame.shape[1]//2])

            screen.blit(surface, (0, 0))
            pygame.display.flip()
            clock.tick(30)

    except KeyboardInterrupt:
        print("\nCTRL+C – zatrzymywanie robota")

    finally:
        # STOP robota
        # send_robot_command(ser, 0, 0, 0)

        out.release()
        cap.release()
        # ser.close()

        print("Zapis wideo zakończony")
        print("Połączenia zamknięte")

if __name__ == "__main__":
    main()
