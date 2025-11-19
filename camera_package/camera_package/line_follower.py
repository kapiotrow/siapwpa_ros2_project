import cv2
import numpy as np
import pytest
from unittest.mock import patch
import time

class LineFollower:
    def __init__(self):
        self.last_vector = [0.5, 0.0, 0.0]  # Default output (searching)
    
    def process_frame(self, frame):
        # Default output
        output_vector = [0.5, 0.0, 0.0]
        
        try:
            if frame is None or frame.size == 0:
                print("Warning: Empty frame.")
                return self.last_vector

            # Convert to HSV color space
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Define yellow range
            lower_yellow = np.array([15, 80, 80])
            upper_yellow = np.array([45, 255, 255])

            # Mask for yellow line
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            height, width, _ = frame.shape
            center_x = width // 2

            if len(contours) > 0:
                # Largest contour (most likely the line)
                contours = sorted(contours, key=cv2.contourArea, reverse=True)
                c = contours[0]
                M = cv2.moments(c)

                if M["m00"] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    # Compute normalized direction: -1 (left) to +1 (right)
                    offset = (cx - center_x) / (center_x)
                    offset = np.clip(offset, -1.0, 1.0)

                    # Visibility = 1 (line seen)
                    visibility = 1.0

                    # Build output vector
                    output_vector = [visibility, offset, 0.0]
                    self.last_vector = output_vector

                    # Visual feedback
                    color = (0, 255, 0)
                    cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)
                    cv2.drawContours(frame, [c], -1, (0, 255, 0), 1)
                    cv2.arrowedLine(frame, (center_x, height - 50),
                                    (int(center_x + offset * 100), height - 120),
                                    color, 4, tipLength=0.4)
                    cv2.putText(frame, f"Dir: {offset:+.2f}", (30, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)
                else:
                    print("Invalid contour (m00=0).")
                    output_vector = [0.5, 0.0, 0.0]

            else:
                # No line found
                print("Line lost — waiting for it to reappear...")
                output_vector = [0.5, 0.0, 0.0]

        except Exception as e:
            print(f"⚠️ Exception: {e}")
            output_vector = [0.5, 0.0, 0.0]

        # Display for debugging
        cv2.imshow("Mask", mask)
        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

        return output_vector

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
        output_vector = lf.process_frame(frame)

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
