import mediapipe as mp
import cv2
import numpy as np

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

pose = mp_pose.Pose(
    model_complexity=1,
    enable_segmentation=False,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.45
)


# Calculates angle between three points
def calculate_angle(a, b, c):
    a = np.array(a)
    b = np.array(b)
    c = np.array(c)

    ba = a - b
    bc = c - b

    cosine_angle = np.dot(ba, bc) / (
        np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-6
    )
    return np.degrees(np.arccos(np.clip(cosine_angle, -1.0, 1.0)))


# Process frames 
def process_frames(frame, origin, locked, active_arm="NONE"):

    origin_locked = locked
    origin_x = origin[0] if locked else 0
    origin_y = origin[1] if locked else 0
    current_active_arm = active_arm
    call_swarm = False
    waist_center = (-1, -1)
    lateral_command = "HOVER" # Default state
    min_angle = 80.0
    max_angle = 150.0
    distance_factor = 0.0

    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    h, w, _ = frame.shape
    frame_copy = frame.copy()

    results_pose = pose.process(rgb_frame)

    if results_pose.pose_landmarks:
        lm = results_pose.pose_landmarks.landmark

        # ---- Landmarks ----

        # Nose - NOSE
        NOSE = lm[mp_pose.PoseLandmark.NOSE]

        # RIGHT ARM
        # Shoulder - RE, Wrist - RW, Elbow - RE
        RS = lm[mp_pose.PoseLandmark.RIGHT_SHOULDER]
        RE = lm[mp_pose.PoseLandmark.RIGHT_ELBOW]
        RW = lm[mp_pose.PoseLandmark.RIGHT_WRIST]
         
        # LEFT ARM
        # Shoulder - LE, Wrist - LW, Elbow - LE
        LS = lm[mp_pose.PoseLandmark.LEFT_SHOULDER]
        LE = lm[mp_pose.PoseLandmark.LEFT_ELBOW]
        LW = lm[mp_pose.PoseLandmark.LEFT_WRIST]
        
        # Hip - LH, RH
        LH = lm[mp_pose.PoseLandmark.LEFT_HIP]
        RH = lm[mp_pose.PoseLandmark.RIGHT_HIP]

        # if left shoulder is to the right of right shoulder, swap the left and right landmarks
        if LS.x > RS.x:
            LS, RS = RS, LS
            LE, RE = RE, LE
            LW, RW = RW, LW


        def px(p): return (int(p.x * w), int(p.y * h))

        # right_elbow_angle: angle between right shoulder, right elbow, and right wrist
        # left_elbow_angle: angle between left shoulder, left elbow, and left wrist
        right_elbow_angle = calculate_angle(px(RS), px(RE), px(RW))
        left_elbow_angle = calculate_angle(px(LS), px(LE), px(LW))
        print(f"right_elbow_angle: {right_elbow_angle}, left_elbow_angle: {left_elbow_angle}")
        # right_shoulder_angle: angle between nose, right shoulder, and right elbow
        # left_shoulder_angle: angle between nose, left shoulder, and left elbow
        right_shoulder_angle = calculate_angle(px(NOSE), px(RS), px(RE))
        left_shoulder_angle = calculate_angle(px(NOSE), px(LS), px(LE))
        print(f"right_shoulder_angle: {right_shoulder_angle}, left_shoulder_angle: {left_shoulder_angle}")
        # ---- Gesture flags ----
        # Lock requires arm to be STRAIGHT (> 150) and UP (wrist above shoulder: RW.y < RS.y)
        pose_right_lock = right_elbow_angle > 150 and RW.y < RS.y and RW.visibility > 0.6 and right_shoulder_angle <= min_angle
        pose_left_lock = left_elbow_angle > 150 and LW.y < LS.y and LW.visibility > 0.6 and left_shoulder_angle <= min_angle
        
        pose_hands_down = False
        if current_active_arm == "RIGHT" and RW.y > RS.y and right_shoulder_angle >= 160:
            pose_hands_down = True
        elif current_active_arm == "LEFT" and LW.y > LS.y and left_shoulder_angle >= 160:
            pose_hands_down = True
        elif current_active_arm == "NONE":
            pose_hands_down = True
        
        if origin_locked:
            if current_active_arm == "RIGHT":
                pose_left_lock = False
            elif current_active_arm == "LEFT":
                pose_right_lock = False



        if origin_locked and pose_hands_down:
            # SAFETY SWITCH: Drop hands to disengage and hover
            origin_locked = False
            current_active_arm = "NONE"
            lateral_command = "HOVER"

        elif not origin_locked:
            if pose_right_lock:
                origin_locked = True
                origin_x, origin_y = px(RS)
                current_active_arm = "RIGHT"

            elif pose_left_lock:
                origin_locked = True
                origin_x, origin_y = px(LS)
                current_active_arm = "LEFT"

        # ---- Steering Update (PROPORTIONAL CONTROL) ----
        if origin_locked and not call_swarm:
            
            # Calibration angles (You can tune these!):
            # min_angle: The angle when arm is straight up near your head.
            # max_angle: The angle when arm is rotated down to horizontal.

            if current_active_arm == "RIGHT":
                # Ensure the arm is still relatively straight to keep driving
                if right_elbow_angle > 140:
                    if right_shoulder_angle > min_angle:
                        # Calculate speed/distance factor (0.0 to 1.0)
                        distance_factor = (right_shoulder_angle - min_angle) / (max_angle - min_angle)
                        distance_factor = np.clip(distance_factor, 0.0, 1.0) # Keep it strictly between 0 and 1
                        print(lateral_command)
                        # Output as a percentage for your ROS node or logic
                        lateral_command = "MOVE RIGHT"
                    else:
                        lateral_command = "HOVER"
                else:
                    lateral_command = "HOVER"

            elif current_active_arm == "LEFT":
                if left_elbow_angle > 140:
                    if left_shoulder_angle > min_angle:
                        distance_factor = (left_shoulder_angle - min_angle) / (max_angle - min_angle)
                        distance_factor = np.clip(distance_factor, 0.0, 1.0)
                        
                        lateral_command = "MOVE LEFT"
                    else:
                        lateral_command = "HOVER"
                else:
                    lateral_command = "HOVER"
        
        if LH.visibility > 0.5 and RH.visibility > 0.5:
            abs_cx = int((LH.x + RH.x) * 0.5 * w)
            abs_cy = int((LH.y + RH.y) * 0.5 * h)
            waist_center = (abs_cx - w // 2, abs_cy - h // 2)
            cv2.circle(frame_copy, (abs_cx, abs_cy), 8, (255, 0, 0), -1)

        # ---- Visualization ----
        mp_drawing.draw_landmarks(
            frame_copy,
            results_pose.pose_landmarks,
            mp_pose.POSE_CONNECTIONS
        )

        # Draw the current mode
        display_text = f"{current_active_arm} ARM ACTIVE" if origin_locked else "IDLE - RAISE ARM STRAIGHT UP"
        
        cv2.putText(frame_copy, display_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame_copy, f"Steering: {lateral_command}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    else:
        origin_locked = False
        current_active_arm = "NONE"
        cv2.putText(frame_copy, "No pose detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    return (
        frame_copy,
        (origin_x, origin_y),
        origin_locked,
        current_active_arm,
        call_swarm,
        waist_center,
        lateral_command,
        distance_factor
    )

def main():
    origin = (0, 0)
    origin_locked = False
    active_arm = "NONE" 

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to read frame from camera.")
            break

        (
            processed_frame,
            origin,
            origin_locked,
            active_arm,
            call_swarm,
            waist_center,
            lateral_command,
            speed_factor
        ) = process_frames(
            frame,
            origin,
            origin_locked,
            active_arm=active_arm,
        )

        cv2.imshow("Drone Controller", processed_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
