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
        lower_yellow = np.array([15, 80, 80])
        upper_yellow = np.array([45, 255, 255])
        
        # Create a mask selecting only yellow parts
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Find contours of the yellow areas
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        if len(contours) > 0:
            # Sort by area and take the largest contour (index 0)
            contours = sorted(contours, key=cv2.contourArea, reverse=True)
            c = contours[1]
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                print(f"CX: {cx}  CY: {cy}")

                # Get frame dimensions
                height, width, _ = frame.shape
                center_x = width // 2
                arrow_start = (center_x, height - 50)
                
                # Determine steering direction
                if cx <= center_x + 50:
                    direction = "Turn Left"
                    arrow_end = (center_x - 50, height - 100)
                    color = (0, 0, 255)  # Red arrow for left
                elif cx >= center_x - 50:
                    direction = "Turn Right"
                    arrow_end = (center_x + 50, height - 100)
                    color = (255, 0, 0)  # Blue arrow for right
                else:
                    direction = "On Track!"
                    arrow_end = (center_x, height - 120)
                    color = (0, 255, 0)  # Green arrow for forward
                
                print(direction)
                
                # Draw the contour and centroid
                cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)
                cv2.drawContours(frame, [c], -1, (0, 255, 0), 1)
                
                # Draw the direction arrow
                cv2.arrowedLine(frame, arrow_start, arrow_end, color, 4, tipLength=0.4)
                cv2.putText(frame, direction, (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 
                            1, color, 2, cv2.LINE_AA)
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
