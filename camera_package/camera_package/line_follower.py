import cv2
import numpy as np
import pytest
from unittest.mock import patch
import time

class LineFollower:
    def __init__(self):
        self.last_direction = "Searching..."  # Keep track of the last known state
    
    def process_frame(self, frame):
        try:
            # Ensure the frame is valid
            if frame is None or frame.size == 0:
                print("Warning: Empty frame received.")
                return

            # Convert to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Yellow color range
            lower_yellow = np.array([15, 80, 80])
            upper_yellow = np.array([45, 255, 255])
            
            # Mask for yellow
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            
            height, width, _ = frame.shape
            center_x = width // 2
            arrow_start = (center_x, height - 50)
            
            if len(contours) > 0:
                # Get largest contour
                contours = sorted(contours, key=cv2.contourArea, reverse=True)
                c = contours[0]
                M = cv2.moments(c)

                if M["m00"] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    print(f"CX: {cx}  CY: {cy}")

                    # Determine direction
                    if cx <= center_x - 20:
                        direction = "Turn Left"
                        arrow_end = (center_x - 50, height - 100)
                        color = (0, 0, 255)
                    elif cx >= center_x + 20:
                        direction = "Turn Right"
                        arrow_end = (center_x + 50, height - 100)
                        color = (255, 0, 0)
                    else:
                        direction = "On Track!"
                        arrow_end = (center_x, height - 120)
                        color = (0, 255, 0)
                    
                    self.last_direction = direction  # Remember last valid direction

                    # Draw visual aids
                    cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)
                    cv2.drawContours(frame, [c], -1, (0, 255, 0), 1)
                    cv2.arrowedLine(frame, arrow_start, arrow_end, color, 4, tipLength=0.4)
                    cv2.putText(frame, direction, (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 
                                1, color, 2, cv2.LINE_AA)
                else:
                    raise ValueError("Zero moment area (m00=0) — invalid contour.")
            
            else:
                # No contour found — line lost
                print("Line lost — waiting for it to reappear...")
                self._draw_search_arrow(frame, arrow_start)
                time.sleep(0.05)  # Small delay to prevent CPU overuse

        except Exception as e:
            print(f"Exception: {e}")
            self._draw_search_arrow(frame, (frame.shape[1] // 2, frame.shape[0] - 50))
        
        # Display results
        cv2.imshow("Mask", mask)
        cv2.imshow("Frame", frame)

    def _draw_search_arrow(self, frame, start_pos):
        """Draw a neutral arrow and text while searching for the line."""
        end_pos = (start_pos[0], start_pos[1] - 80)
        cv2.arrowedLine(frame, start_pos, end_pos, (0, 255, 255), 4, tipLength=0.4)
        cv2.putText(frame, "Searching...", (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 
                    1, (0, 255, 255), 2, cv2.LINE_AA)

def test_line_follower_manual_display():
    """
    Manual visual test — opens OpenCV windows to display frames.
    Press 'q' to quit.
    """
    lf = LineFollower()
    path = "/home/developer/ros2_ws/src/camera_package/camera_package/test_video.avi"
    cap = cv2.VideoCapture(path)
    assert cap.isOpened(), "Failed to open test video"

    print("\nManual Line Follower Test Started.")
    print("Press 'q' in the video window to stop.\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("End of video reached.")
            break

        lf.process_frame(frame)

        # small wait to make display visible
        cv2.waitKey(10)

    cap.release()
    cv2.destroyAllWindows()
