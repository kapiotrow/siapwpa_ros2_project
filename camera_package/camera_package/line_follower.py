import cv2
import numpy as np
from .Controllers import PIDController, LQRController
from .FrameProcessors import BaseLineProcessor, LineKalmanProcessor
import pytest

class LineFollower:
    def __init__(self):
        self.processor = LineKalmanProcessor()
        self.controller = PIDController()
        self.controller.attach_processor(self.processor)
        print(f"Processor: {type(self.processor)}")
        print(f"Controller: {type(self.controller)}")

    def update(self, frame):
        """
        Full pipeline: Frame → Processor → PIDControl
        """
        return self.controller.compute(frame)




def test_line_follower_manual_display():
    """
    Manual visual test — opens OpenCV windows to display frames and prints output vectors.
    Press 'q' to quit.
    """
    lf = LineFollower()
    path = "/home/developer/ros2_ws/src/camera_package/camera_package/test_video.avi"
    cap = cv2.VideoCapture(path)
    assert cap.isOpened(), "Failed to open test video"

    print("\nManual Line Follower Test Started.")
    print("Press 'q' in the video window to stop.\n")

    frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("End of video reached.")
            break

        # Process frame and get output vector
        output_vector = lf.update(frame)

        # Print output for debugging
        print(f"Frame {frame_count:04d} — Output: {output_vector}")

        frame_count += 1

        # Wait for 'q' key to quit early
        key = cv2.waitKey(10) & 0xFF
        if key == ord('q'):
            print("Manual stop requested.")
            break

    cap.release()
    cv2.destroyAllWindows()
    print("Test completed successfully.\n")

