import cv2
import numpy as np
import pytest
from unittest.mock import patch

class LineFollower:
    def __init__(self):
        pass
    
    def process_frame(self, frame):
      low_b = np.uint8([5,5,5])
      high_b = np.uint8([0,0,0])
      mask = cv2.inRange(frame, high_b, low_b)
      contours, hierarchy = cv2.findContours(mask, 1, cv2.CHAIN_APPROX_NONE)
      if len(contours) > 0 :
          c = max(contours, key=cv2.contourArea)
          M = cv2.moments(c)
          if M["m00"] !=0 :
              cx = int(M['m10']/M['m00'])
              cy = int(M['m01']/M['m00'])
              print("CX : "+str(cx)+"  CY : "+str(cy))
              if cx >= 120 :
                  print("Turn Left")
              if cx < 120 and cx > 40 :
                  print("On Track!")
              if cx <=40 :
                  print("Turn Right")
              cv2.circle(frame, (cx,cy), 5, (255,255,255), -1)
      else :
          print("I don't see the line")
      cv2.drawContours(frame, c, -1, (0,255,0), 1)
      cv2.imshow("Mask",mask)
      cv2.imshow("Frame",frame)


@pytest.fixture
def sample_video(tmp_path):
    """Creates a short synthetic video with a black line moving across white background."""
    video_path = "/home/developer/ros2_ws/src/camera_package/camera_package/test_video.avi"
    frame_width, frame_height = 160, 120
    out = cv2.VideoWriter(str(video_path),
                          cv2.VideoWriter_fourcc(*"XVID"),
                          10, (frame_width, frame_height))

    # moving black line for testing
    for x in range(20, 140, 10):
        frame = np.ones((frame_height, frame_width, 3), dtype=np.uint8) * 255
        cv2.line(frame, (x, 60), (x, 80), (0, 0, 0), 5)
        out.write(frame)
    out.release()
    return str(video_path)

def test_line_follower_manual_display(sample_video):
    """
    Manual visual test — opens OpenCV windows to display frames.
    Press 'q' to quit.
    """
    lf = LineFollower()
    cap = cv2.VideoCapture(sample_video)
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
        if cv2.waitKey(100) & 0xFF == ord('q'):
            print("Manual test interrupted by user.")
            break

    cap.release()
    cv2.destroyAllWindows()
