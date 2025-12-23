import mediapipe as mp
import cv2
import numpy as np

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

hands = mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.5
)

pose = mp_pose.Pose(  # lightweight single-person pose
    model_complexity=1,
    enable_segmentation=False,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)


def process_frames(frame, origin, locked, distance, tracking_vertical=False):
    """Gesture-based tracking with swarm command support for ROS2 drone control.
       Now also returns waist center in image-centered pixel coords (dx, dy),
       or (-1, -1) if unavailable.
    """
    origin_locked = locked
    origin_x = origin[0] if locked else 0
    origin_y = origin[1] if locked else 0
    current_tracking_vertical = tracking_vertical
    call_swarm = False

    # Default waist center relative to frame center (sentinel)
    waist_center = (-1, -1)

    # --- Common pre-processing ---
    frame = cv2.flip(frame, -1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    h, w, _ = frame.shape
    frame_copy = frame.copy()

    # --- Hand processing (unchanged logic) ---
    results_hands = hands.process(rgb_frame)

    if results_hands.multi_hand_landmarks:
        for hand_landmarks in results_hands.multi_hand_landmarks:
            palm_landmark = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
            palm_x = int(palm_landmark.x * w)
            palm_y = int(palm_landmark.y * h)

            finger_tips = [8, 12, 16, 20]
            finger_pips = [6, 10, 14, 18]

            # Gesture detection
            fist_detected = all(
                hand_landmarks.landmark[tip].y > hand_landmarks.landmark[pip].y
                for tip, pip in zip(finger_tips, finger_pips)
            )

            index_up = hand_landmarks.landmark[8].y < hand_landmarks.landmark[6].y
            middle_up = hand_landmarks.landmark[12].y < hand_landmarks.landmark[10].y
            pinky_up = hand_landmarks.landmark[20].y < hand_landmarks.landmark[18].y

            # INDEX ONLY UP: Reset (middle, ring, pinky curled)
            others_curled = all(
                hand_landmarks.landmark[tip].y > hand_landmarks.landmark[pip].y
                for tip, pip in zip(finger_tips[1:], finger_pips[1:])
            )
            index_only_up = index_up and others_curled

            # INDEX + PINKY UP: Vertical mode (middle+ring curled)
            middle_ring_curled = all(
                hand_landmarks.landmark[tip].y > hand_landmarks.landmark[pip].y
                for tip, pip in zip([12, 16], [10, 14])
            )
            index_pinky_up = index_up and pinky_up and middle_ring_curled

            # INDEX + MIDDLE UP: Swarm (ring+pinky curled)
            ring_pinky_curled = all(
                hand_landmarks.landmark[tip].y > hand_landmarks.landmark[pip].y
                for tip, pip in zip([16, 20], [14, 18])
            )
            index_middle_up = index_up and middle_up and ring_pinky_curled

            print(f"Index:{index_up} Middle:{middle_up} Ring/Pringy:{ring_pinky_curled} Swarm:{index_middle_up}")

            # State machine - FIXED ORDER (swarm before reset)
            if not origin_locked:
                if fist_detected:
                    origin_locked = True
                    origin_x = palm_x
                    origin_y = palm_y
                    current_tracking_vertical = False
                    distance = 0
                    print("FIST LOCKED - HORIZONTAL MODE")
                elif index_pinky_up:
                    origin_locked = True
                    origin_x = palm_x
                    origin_y = palm_y
                    current_tracking_vertical = True
                    distance = 0
                    print("INDEX+PINKY LOCKED - VERTICAL MODE")
            else:
                # Check SWARM FIRST when locked
                if index_middle_up:
                    call_swarm = True
                    print("🚀 INDEX+MIDDLE SWARM DETECTED!")
                # Then check reset
                elif index_only_up:
                    origin_locked = False
                    origin_x = 0
                    origin_y = 0
                    distance = 0
                    current_tracking_vertical = False
                    print("INDEX ONLY - RESET")

            # Update distance
            if origin_locked:
                distance = (
                    palm_y - origin_y
                    if current_tracking_vertical
                    else palm_x - origin_x
                )

            # Visualization (unchanged)
            mp_drawing.draw_landmarks(
                frame_copy,
                hand_landmarks,
                mp_hands.HAND_CONNECTIONS
            )

            all_tips = [8, 12, 16, 20]
            all_pips = [6, 10, 14, 18]
            for tip_idx, pip_idx in zip(all_tips, all_pips):
                tip_pos = (
                    int(hand_landmarks.landmark[tip_idx].x * w),
                    int(hand_landmarks.landmark[tip_idx].y * h)
                )
                is_up = hand_landmarks.landmark[tip_idx].y < hand_landmarks.landmark[pip_idx].y

                if tip_idx in [8, 12, 20]:
                    color = (0, 255, 0) if is_up else (0, 0, 255)
                else:
                    color = (0, 255, 0) if not is_up else (0, 0, 255)

                cv2.circle(frame_copy, tip_pos, 8, color, -1)

            if origin_locked:
                mode_text = "VERTICAL" if current_tracking_vertical else "HORIZONTAL"
                cv2.line(frame_copy, (origin_x, 0), (origin_x, h), (0, 255, 0), 3)
                cv2.line(frame_copy, (0, origin_y), (w, origin_y), (0, 255, 0), 3)
                cv2.circle(frame_copy, (palm_x, palm_y), 12, (0, 255, 0), -1)
                cv2.circle(frame_copy, (origin_x, origin_y), 10, (255, 255, 0), -1)

                status_lines = [
                    f'{mode_text}: {distance:.1f}px',
                    'INDEX UP = RESET',
                    f'INDEX+MIDDLE = SWARM{" ✓" if call_swarm else ""}'
                ]
                for i, text in enumerate(status_lines):
                    color = (0, 255, 0) if '✓' in text else (0, 255, 255)
                    cv2.putText(
                        frame_copy,
                        text,
                        (10, 30 + i * 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        color,
                        2
                    )
            else:
                ready_texts = []
                if fist_detected:
                    ready_texts.append('FIST READY! (Horizontal)')
                if index_pinky_up:
                    ready_texts.append('INDEX+PINKY READY! (Vertical)')
                if index_middle_up:
                    ready_texts.append('INDEX+MIDDLE = SWARM')

                cv2.putText(
                    frame_copy,
                    'FIST=Horizontal | INDEX+PINKY=Vertical',
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2
                )
                for i, text in enumerate(ready_texts):
                    cv2.putText(
                        frame_copy,
                        text,
                        (10, 70 + i * 25),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2
                    )
    else:
        cv2.putText(
            frame_copy,
            'No hand detected',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )

    # --- Pose processing for waist center (relative to frame center) ---
    results_pose = pose.process(rgb_frame)

    if results_pose.pose_landmarks:
        lm = results_pose.pose_landmarks.landmark
        # LEFT_HIP = 23, RIGHT_HIP = 24 for BlazePose
        left_hip = lm[mp_pose.PoseLandmark.LEFT_HIP]
        right_hip = lm[mp_pose.PoseLandmark.RIGHT_HIP]

        # Optional visibility check to ensure a standing person is reasonably detected
        if left_hip.visibility > 0.5 and right_hip.visibility > 0.5:
            # Absolute pixel coordinates
            abs_cx = int((left_hip.x + right_hip.x) * 0.5 * w)
            abs_cy = int((left_hip.y + right_hip.y) * 0.5 * h)

            # Convert to coordinates relative to frame center
            cx_rel = abs_cx - w // 2
            cy_rel = abs_cy - h // 2
            waist_center = (cx_rel, cy_rel)

            # Visualize on the frame in absolute coordinates
            cv2.circle(frame_copy, (abs_cx, abs_cy), 10, (255, 0, 0), -1)
            cv2.putText(
                frame_copy,
                'WAIST',
                (abs_cx - 30, abs_cy - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 0, 0),
                2
            )

            # Optional: draw image center and a line from center to waist
            cv2.circle(frame_copy, (w // 2, h // 2), 5, (0, 255, 255), -1)
            cv2.line(frame_copy, (w // 2, h // 2), (abs_cx, abs_cy), (255, 0, 0), 2)

    # Final return now includes waist_center in image-centered pixel coords
    return (
        frame_copy,
        (origin_x, origin_y),
        origin_locked,
        distance,
        current_tracking_vertical,
        call_swarm,
        waist_center
    )
