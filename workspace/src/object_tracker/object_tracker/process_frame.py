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


def process_frames(frame, origin, locked, active_arm="NONE"):

    origin_locked = locked
    origin_x = origin[0] if locked else 0
    origin_y = origin[1] if locked else 0
    current_active_arm = active_arm
    call_swarm = False
    waist_center = (-1, -1)
    lateral_command = "HOVER" # Default state

    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    h, w, _ = frame.shape
    frame_copy = frame.copy()

    results_pose = pose.process(rgb_frame)

    if results_pose.pose_landmarks:
        lm = results_pose.pose_landmarks.landmark

        # ---- Landmarks ----
        RS = lm[mp_pose.PoseLandmark.RIGHT_SHOULDER]
        RE = lm[mp_pose.PoseLandmark.RIGHT_ELBOW]
        RW = lm[mp_pose.PoseLandmark.RIGHT_WRIST]

        LS = lm[mp_pose.PoseLandmark.LEFT_SHOULDER]
        LE = lm[mp_pose.PoseLandmark.LEFT_ELBOW]
        LW = lm[mp_pose.PoseLandmark.LEFT_WRIST]

        LH = lm[mp_pose.PoseLandmark.LEFT_HIP]
        RH = lm[mp_pose.PoseLandmark.RIGHT_HIP]

        # ---------- FIX 1: LEFT / RIGHT CONSISTENCY ----------
        if LS.x > RS.x:
            LS, RS = RS, LS
            LE, RE = RE, LE
            LW, RW = RW, LW

        def px(p): return (int(p.x * w), int(p.y * h))

        # ---- Angles ----
        # (We still need angles to know if you are bending your arm to activate it)
        right_angle = calculate_angle(px(RS), px(RE), px(RW))
        left_angle = calculate_angle(px(LS), px(LE), px(LW))

        # ---- Gesture flags ----
        pose_right_lock = right_angle < 130 and RW.visibility > 0.6
        pose_left_lock = left_angle < 130 and LW.visibility > 0.6
        
        # HANDS DOWN FLAG (Wrists below hips, arms mostly straight)
        pose_hands_down = (
            right_angle > 140 and left_angle > 140 and
            RW.y > LH.y and LW.y > LH.y
        )

        pose_swarm = (
            right_angle < 120 and left_angle < 120 and
            RS.y < RW.y < LH.y and
            LS.y < LW.y < LH.y and
            RW.visibility > 0.6 and LW.visibility > 0.6
        )

        # ---------- FIX 2: PREVENT AXIS SWITCHING ----------
        if origin_locked:
            if current_active_arm == "RIGHT":
                pose_left_lock = False
            elif current_active_arm == "LEFT":
                pose_right_lock = False

        # ================= STATE MACHINE (PRIORITY) =================

        if pose_swarm:
            lateral_command = "HOVER"
            call_swarm = True

        elif pose_hands_down:
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

        # ---- Steering Update ----
        if origin_locked and not call_swarm:
            deadzone1 = 0.10
            deadzone2 = 0.20  

            if current_active_arm == "RIGHT":
                # Steering logic based on Right Arm
                if RW.x > (RS.x + deadzone2):
                    lateral_command = "MOVE RIGHT"

            elif current_active_arm == "LEFT":
                # Steering logic based on Left Arm
               
                if LW.x < (LS.x - deadzone2):
                    lateral_command = "MOVE LEFT"

        # ---- Visualization ----
        mp_drawing.draw_landmarks(
            frame_copy,
            results_pose.pose_landmarks,
            mp_pose.POSE_CONNECTIONS
        )

        # Draw the current mode
        display_text = f"{current_active_arm} ARM ACTIVE" if origin_locked else "IDLE - RAISE ARM TO CONTROL"
        
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
        lateral_command # Passing this out so you can eventually send it to ROS
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
            lateral_command
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