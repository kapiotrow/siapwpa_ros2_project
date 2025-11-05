import cv2
import numpy as np
import pytest
from unittest.mock import patch

class LineFollower:
    def __init__(self):
        pass
    
    def process_frame(self, frame):
        # Convert the image from BGR (OpenCV default) to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define yellow color range in HSV
        # (tune these if lighting varies)
        lower_yellow = np.array([15, 80, 80])
        upper_yellow = np.array([45, 255, 255])
        
        # Create a mask selecting only yellow parts
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Find contours of the yellow areas
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        if len(contours) > 0:
            # Get the largest contour (most likely the line)
            contours = sorted(contours, key=cv2.contourArea, reverse=True)
            c = contours[1]
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                print(f"CX: {cx}  CY: {cy}")
                
                # Simple steering logic
                if cx >= 120:
                    print("Turn Left")
                elif 40 < cx < 120:
                    print("On Track!")
                else:
                    print("Turn Right")
                
                # Draw visual markers
                cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)
                cv2.drawContours(frame, [c], -1, (0, 255, 0), 1)
        else:
            print("I don't see the yellow line")
        
        # Show the images
        cv2.imshow("Mask", mask)
        cv2.imshow("Frame", frame)

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
