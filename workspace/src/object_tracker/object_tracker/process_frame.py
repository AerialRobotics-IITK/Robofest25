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


def map_angle_to_distance(angle, min_angle=60, max_angle=160, max_dist=300):
    angle = np.clip(angle, min_angle, max_angle)
    return ((angle - min_angle) / (max_angle - min_angle)) * max_dist


def process_frames(frame, origin, locked, distance, tracking_vertical=False):

    origin_locked = locked
    origin_x = origin[0] if locked else 0
    origin_y = origin[1] if locked else 0
    current_tracking_vertical = tracking_vertical
    call_swarm = False
    waist_center = (-1, -1)

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
        right_angle = calculate_angle(px(RS), px(RE), px(RW))
        left_angle = calculate_angle(px(LS), px(LE), px(LW))

        # ---- Gesture flags ----
        pose_horizontal_lock = right_angle < 130 and RW.visibility > 0.6
        pose_vertical_lock = left_angle < 130 and LW.visibility > 0.6
        pose_reset = (
            origin_locked and
            right_angle > 150 and left_angle > 150 and
            RW.y > LH.y and LW.y > LH.y and
            RW.visibility > 0.6 and LW.visibility > 0.6
        )

        pose_swarm = (
            right_angle < 120 and left_angle < 120 and
            RS.y < RW.y < LH.y and
            LS.y < LW.y < LH.y and
            RW.visibility > 0.6 and LW.visibility > 0.6
        )

        # ---------- FIX 2: PREVENT AXIS SWITCHING ----------
        if origin_locked:
            if current_tracking_vertical:
                pose_horizontal_lock = False
            else:
                pose_vertical_lock = False

        # ================= STATE MACHINE (PRIORITY) =================

        if pose_swarm:
            distance = 0
            call_swarm = True
            print("POSE SWARM DETECTED (PRIORITY)")

        elif origin_locked and pose_reset:
            origin_locked = False
            origin_x = 0
            origin_y = 0
            distance = 0
            current_tracking_vertical = False
            print("BOTH HANDS UP - RESET")

        elif not origin_locked:
            if pose_horizontal_lock:
                origin_locked = True
                origin_x, origin_y = px(RS)
                current_tracking_vertical = False
                distance = 0
                print("RIGHT ARM LOCKED - HORIZONTAL MODE")

            elif pose_vertical_lock:
                origin_locked = True
                origin_x, origin_y = px(LS)
                current_tracking_vertical = True
                distance = 0
                print("LEFT ARM LOCKED - VERTICAL MODE")

        # ---- Distance update ----
        if origin_locked and not call_swarm:
            angle = left_angle if current_tracking_vertical else right_angle
            distance = map_angle_to_distance(angle)

        # ---- Waist center ----
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

        mode = "VERTICAL" if current_tracking_vertical else "HORIZONTAL"
        cv2.putText(
            frame_copy,
            f"{mode}: {distance:.1f}" if origin_locked else
            "RIGHT ARM = H | LEFT ARM = V | BOTH UP = RESET",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

    else:
        distance = 0
        origin_locked = False
        cv2.putText(
            frame_copy,
            "No pose detected",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2
        )

    return (
        frame_copy,
        (origin_x, origin_y),
        origin_locked,
        distance,
        current_tracking_vertical,
        call_swarm,
        waist_center
    )