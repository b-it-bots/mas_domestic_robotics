#Author - Shrikar Nakhye
import cv2
import mediapipe as mp

mp_pose = mp.solutions.pose
pose = mp_pose.Pose()
cap = cv2.VideoCapture(0)

FRAME_WIDTH = 640
CENTER_THRESHOLD = 100

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to access the camera.")
            break


        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)


        results = pose.process(rgb_frame)


        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark
            nose = landmarks[mp_pose.PoseLandmark.NOSE]

            x = int(nose.x * FRAME_WIDTH)
            y = int(nose.y * frame.shape[0])

            cv2.circle(frame, (x, y), 10, (0, 255, 0), -1)

            center_x = FRAME_WIDTH // 2
            if x < center_x - CENTER_THRESHOLD:
                print("Action: Move Left")
            elif x > center_x + CENTER_THRESHOLD:
                print("Action: Move Right")
            else:
                print("Action: Move Forward")
        else:
            print("Action: Stop - Person not detected")

        cv2.imshow("Person Tracking", frame)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
