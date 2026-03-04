import mediapipe as mp
import cv2
import numpy as np

# ================= INIT =================

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

pose = mp_pose.Pose(
    model_complexity=1,
    enable_segmentation=False,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.45
)


# ================= UTILS =================

def calculate_angle(a, b, c):
    """
    Angle at point b between points a-b-c
    """
    a = np.array(a)
    b = np.array(b)
    c = np.array(c)

    ba = a - b
    bc = c - b

    cosine = np.dot(ba, bc) / (
        np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-6
    )

    return np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0)))


def map_angle_to_distance(
    angle,
    min_angle=60,
    max_angle=160,
    max_dist=300
):
    """
    Convert shoulder angle to distance
    """
    angle = np.clip(angle, min_angle, max_angle)

    return ((angle - min_angle) /
            (max_angle - min_angle)) * max_dist


# ================= MAIN =================

def process_frames(frame, origin, locked, distance):

    origin_locked = locked
    origin_x, origin_y = origin
    call_swarm = False
    waist_center = (-1, -1)

    # Flip for mirror effect
    frame = cv2.flip(frame, 1)

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    h, w, _ = frame.shape

    frame_copy = frame.copy()

    results = pose.process(rgb)

    # ================= NO POSE =================

    if not results.pose_landmarks:

        origin_locked = False
        distance = 0

        cv2.putText(
            frame_copy,
            "No Pose Detected",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )

        return (
            frame_copy,
            (0, 0),
            origin_locked,
            distance,
            call_swarm,
            waist_center
        )

    # ================= LANDMARKS =================

    lm = results.pose_landmarks.landmark

    RS = lm[mp_pose.PoseLandmark.RIGHT_SHOULDER]
    RE = lm[mp_pose.PoseLandmark.RIGHT_ELBOW]
    RW = lm[mp_pose.PoseLandmark.RIGHT_WRIST]

    LS = lm[mp_pose.PoseLandmark.LEFT_SHOULDER]
    LE = lm[mp_pose.PoseLandmark.LEFT_ELBOW]
    LW = lm[mp_pose.PoseLandmark.LEFT_WRIST]

    RH = lm[mp_pose.PoseLandmark.RIGHT_HIP]
    LH = lm[mp_pose.PoseLandmark.LEFT_HIP]

    # Pixel conversion
    def px(p):
        return int(p.x * w), int(p.y * h)

    # ================= ANGLES =================
    # Elbow - Shoulder - Hip

    right_angle = calculate_angle(
        px(RE), px(RS), px(RH)
    )

    left_angle = calculate_angle(
        px(LE), px(LS), px(LH)
    )

    # ================= GESTURES =================

    # Lock: Both hands raised
    pose_lock = (
        RW.y < RS.y and
        LW.y < LS.y and
        right_angle > 140 and
        left_angle > 140 and
        RW.visibility > 0.6 and
        LW.visibility > 0.6
    )

    # Reset: Both hands down
    pose_reset = (
        RW.y > RH.y and
        LW.y > LH.y and
        right_angle < 100 and
        left_angle < 100 and
        RW.visibility > 0.6 and
        LW.visibility > 0.6
    )

    # Swarm (optional gesture)
    pose_swarm = (
        right_angle < 120 and
        left_angle < 120 and
        RS.y < RW.y < RH.y and
        LS.y < LW.y < LH.y and
        RW.visibility > 0.6 and
        LW.visibility > 0.6
    )

    # ================= STATE MACHINE =================

    # Swarm priority
    if pose_swarm:
        distance = 0
        call_swarm = True
        print("SWARM MODE")

    # Reset
    elif origin_locked and pose_reset:

        origin_locked = False
        origin_x, origin_y = 0, 0
        distance = 0

        print("SYSTEM RESET")

    # Lock
    elif not origin_locked and pose_lock:

        origin_locked = True

        # Shoulder midpoint
        sx = int((RS.x + LS.x) * 0.5 * w)
        sy = int((RS.y + LS.y) * 0.5 * h)

        origin_x, origin_y = sx, sy
        distance = 0

        print("SYSTEM LOCKED")

    # ================= CONTROL =================

    if origin_locked and not call_swarm:

        avg_angle = (left_angle + right_angle) * 0.5

        distance = map_angle_to_distance(avg_angle)

    # ================= WAIST CENTER =================

    if RH.visibility > 0.5 and LH.visibility > 0.5:

        cx = int((RH.x + LH.x) * 0.5 * w)
        cy = int((RH.y + LH.y) * 0.5 * h)

        waist_center = (cx - w // 2, cy - h // 2)

        cv2.circle(
            frame_copy,
            (cx, cy),
            8,
            (255, 0, 0),
            -1
        )

    # ================= DRAW =================

    mp_drawing.draw_landmarks(
        frame_copy,
        results.pose_landmarks,
        mp_pose.POSE_CONNECTIONS
    )

    if origin_locked:
        text = f"LOCKED | Distance: {distance:.1f}"
    else:
        text = "Raise BOTH hands to LOCK"

    cv2.putText(
        frame_copy,
        text,
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2
    )

    return (
        frame_copy,
        (origin_x, origin_y),
        origin_locked,
        distance,
        call_swarm,
        waist_center
    )
