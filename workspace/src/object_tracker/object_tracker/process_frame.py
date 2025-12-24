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

pose = mp_pose.Pose(
    model_complexity=1,
    enable_segmentation=False,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

def process_frames(frame, origin, locked, distance, tracking_vertical=False):
    """
    Complete Gesture Tracking with ROI Upscaling for High-Altitude Drone Flight.
    Combines original state machine with dynamic 'Digital Zoom' logic.
    """
    origin_locked = locked
    origin_x = origin[0] if locked else 0
    origin_y = origin[1] if locked else 0
    current_tracking_vertical = tracking_vertical
    call_swarm = False
    waist_center = (-1, -1)

    # --- Pre-processing ---
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    h, w, _ = frame.shape
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_copy = frame.copy()

    # --- 1. Pose processing FIRST (to find the person for the ROI) ---
    results_pose = pose.process(rgb_frame)
    
    # Default to full frame if pose is not detected
    hand_roi_rgb = rgb_frame 
    offset_x, offset_y = 0, 0
    roi_w, roi_h = w, h

    if results_pose.pose_landmarks:
        lm = results_pose.pose_landmarks.landmark
        left_hip = lm[mp_pose.PoseLandmark.LEFT_HIP]
        right_hip = lm[mp_pose.PoseLandmark.RIGHT_HIP]
        
        if left_hip.visibility > 0.5 and right_hip.visibility > 0.5:
            # Waist Center for ROS2 Drone Control
            abs_cx = int((left_hip.x + right_hip.x) * 0.5 * w)
            abs_cy = int((left_hip.y + right_hip.y) * 0.5 * h)
            waist_center = (abs_cx - w // 2, abs_cy - h // 2)

            # --- 2. Calculate Dynamic ROI (Digital Zoom) ---
            l_sh = lm[mp_pose.PoseLandmark.LEFT_SHOULDER]
            r_sh = lm[mp_pose.PoseLandmark.RIGHT_SHOULDER]
            
            # Find boundaries of the torso
            x_min = min(l_sh.x, r_sh.x, left_hip.x, right_hip.x)
            x_max = max(l_sh.x, r_sh.x, left_hip.x, right_hip.x)
            y_min = min(l_sh.y, r_sh.y)
            y_max = max(left_hip.y, right_hip.y)

            # Add padding (30%) to ensure hands are in frame even when raised
            padding = 0.3
            box_x1 = max(0, int((x_min - padding) * w))
            box_y1 = max(0, int((y_min - padding * 1.5) * h)) # Extra top padding
            box_x2 = min(w, int((x_max + padding) * w))
            box_y2 = min(h, int((y_max + padding) * h))

            # Crop for hand detection
            hand_roi_rgb = rgb_frame[box_y1:box_y2, box_x1:box_x2]
            offset_x, offset_y = box_x1, box_y1
            roi_h, roi_w, _ = hand_roi_rgb.shape

            # Visual debug: draw the ROI box
            cv2.rectangle(frame_copy, (box_x1, box_y1), (box_x2, box_y2), (255, 255, 255), 1)

    # --- 3. Hand processing on the CROP ---
    results_hands = hands.process(hand_roi_rgb)

    if results_hands.multi_hand_landmarks:
        for hand_landmarks in results_hands.multi_hand_landmarks:
            # Scale coordinates back to full-frame for drone control
            wrist_lm = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
            palm_x = int(wrist_lm.x * roi_w) + offset_x
            palm_y = int(wrist_lm.y * roi_h) + offset_y

            # Finger detection logic
            finger_tips = [8, 12, 16, 20]
            finger_pips = [6, 10, 14, 18]

            fist_detected = all(hand_landmarks.landmark[tip].y > hand_landmarks.landmark[pip].y for tip, pip in zip(finger_tips, finger_pips))
            index_up = hand_landmarks.landmark[8].y < hand_landmarks.landmark[6].y
            middle_up = hand_landmarks.landmark[12].y < hand_landmarks.landmark[10].y
            pinky_up = hand_landmarks.landmark[20].y < hand_landmarks.landmark[18].y

            # Compound Gestures
            others_curled = all(hand_landmarks.landmark[tip].y > hand_landmarks.landmark[pip].y for tip, pip in zip(finger_tips[1:], finger_pips[1:]))
            index_only_up = index_up and others_curled

            middle_ring_curled = all(hand_landmarks.landmark[tip].y > hand_landmarks.landmark[pip].y for tip, pip in zip([12, 16], [10, 14]))
            index_pinky_up = index_up and pinky_up and middle_ring_curled

            ring_pinky_curled = all(hand_landmarks.landmark[tip].y > hand_landmarks.landmark[pip].y for tip, pip in zip([16, 20], [14, 18]))
            index_middle_up = index_up and middle_up and ring_pinky_curled

            # State Machine
            if not origin_locked:
                if fist_detected:
                    origin_locked, origin_x, origin_y = True, palm_x, palm_y
                    current_tracking_vertical, distance = False, 0
                elif index_pinky_up:
                    origin_locked, origin_x, origin_y = True, palm_x, palm_y
                    current_tracking_vertical, distance = True, 0
            else:
                if index_middle_up:
                    call_swarm = True
                elif index_only_up:
                    origin_locked, origin_x, origin_y, distance, current_tracking_vertical = False, 0, 0, 0, False

            if origin_locked:
                distance = palm_y - origin_y if current_tracking_vertical else palm_x - origin_x

            # --- 4. Drawing Landmarks & Visual Feedback ---
            # Map landmarks back to full frame for mp_drawing
            for lm in hand_landmarks.landmark:
                lm.x = (lm.x * roi_w + offset_x) / w
                lm.y = (lm.y * roi_h + offset_y) / h
            
            mp_drawing.draw_landmarks(frame_copy, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            # Colored tip circles
            for tip_idx, pip_idx in zip(finger_tips, finger_pips):
                tip_pos = (int(hand_landmarks.landmark[tip_idx].x * w), int(hand_landmarks.landmark[tip_idx].y * h))
                is_up = hand_landmarks.landmark[tip_idx].y < hand_landmarks.landmark[pip_idx].y
                color = (0, 255, 0) if (is_up if tip_idx in [8, 12, 20] else not is_up) else (0, 0, 255)
                cv2.circle(frame_copy, tip_pos, 8, color, -1)

            if origin_locked:
                mode_text = "VERTICAL" if current_tracking_vertical else "HORIZONTAL"
                cv2.line(frame_copy, (origin_x, 0), (origin_x, h), (0, 255, 0), 3)
                cv2.line(frame_copy, (0, origin_y), (w, origin_y), (0, 255, 0), 3)
                cv2.circle(frame_copy, (palm_x, palm_y), 12, (0, 255, 0), -1)
                cv2.circle(frame_copy, (origin_x, origin_y), 10, (255, 255, 0), -1)
                
                status_lines = [f'{mode_text}: {distance:.1f}px', 'INDEX UP = RESET', f'INDEX+MIDDLE = SWARM{" ✓" if call_swarm else ""}']
                for i, text in enumerate(status_lines):
                    cv2.putText(frame_copy, text, (10, 30 + i * 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if '✓' in text else (0, 255, 255), 2)
            else:
                cv2.putText(frame_copy, 'FIST=Horizontal | INDEX+PINKY=Vertical', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    else:
        cv2.putText(frame_copy, 'No hand detected', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # --- 5. Pose Visualization (Waist) ---
    if results_pose.pose_landmarks and waist_center != (-1, -1):
        abs_cx, abs_cy = waist_center[0] + w // 2, waist_center[1] + h // 2
        cv2.circle(frame_copy, (abs_cx, abs_cy), 10, (255, 0, 0), -1)
        cv2.putText(frame_copy, 'WAIST', (abs_cx - 30, abs_cy - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        cv2.circle(frame_copy, (w // 2, h // 2), 5, (0, 255, 255), -1)
        cv2.line(frame_copy, (w // 2, h // 2), (abs_cx, abs_cy), (255, 0, 0), 2)

    return frame_copy, (origin_x, origin_y), origin_locked, distance, current_tracking_vertical, call_swarm, waist_center