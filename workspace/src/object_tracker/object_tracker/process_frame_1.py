import mediapipe as mp
import cv2
import numpy as np

# Initialize MediaPipe Pose components
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

# Configure the Pose estimation model
# Complexity 1 is a good balance between speed and accuracy for real-time webcam use
pose = mp_pose.Pose(
    model_complexity=1,
    enable_segmentation=False,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.45
)

def calculate_angle(a, b, c):
    """
    Calculates the angle (in degrees) formed by three points.
    Typically used for joint angles (e.g., Shoulder -> Elbow -> Wrist).
    """
    a = np.array(a) # First point (e.g., Shoulder)
    b = np.array(b) # Mid point (e.g., Elbow)
    c = np.array(c) # End point (e.g., Wrist)

    # Create vectors for the two segments intersecting at point 'b'
    ba = a - b
    bc = c - b

    # Calculate the cosine of the angle using the dot product
    # Added 1e-6 to the denominator to prevent division by zero
    cosine_angle = np.dot(ba, bc) / (
        np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-6
    )
    
    # Clip the value between -1.0 and 1.0 to prevent arccos from returning NaN 
    # due to minor floating-point inaccuracies
    return np.degrees(np.arccos(np.clip(cosine_angle, -1.0, 1.0)))


def process_frames(frame, origin, locked, active_arm="NONE"):
    """
    Processes a single video frame to detect poses, interpret gestures, 
    and output drone control commands.
    """
    # --- 1. STATE INITIALIZATION ---
    origin_locked = locked
    origin_x = origin[0] if locked else 0
    origin_y = origin[1] if locked else 0
    current_active_arm = active_arm
    
    call_swarm = False
    waist_center = (-1, -1)
    lateral_command = "HOVER" # Default safe state

    # --- 2. IMAGE PREPROCESSING ---
    # Flip horizontally for a "mirror" effect (makes left/right movement intuitive)
    frame = cv2.flip(frame, 1)
    # MediaPipe requires RGB color space, while OpenCV uses BGR by default
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    h, w, _ = frame.shape
    frame_copy = frame.copy()

    # Run the image through the MediaPipe Pose model
    results_pose = pose.process(rgb_frame)

    # --- 3. POSE ANALYSIS & LOGIC ---
    if results_pose.pose_landmarks:
        lm = results_pose.pose_landmarks.landmark

        # Extract Key Anatomical Landmarks
        RS = lm[mp_pose.PoseLandmark.RIGHT_SHOULDER]
        RE = lm[mp_pose.PoseLandmark.RIGHT_ELBOW]
        RW = lm[mp_pose.PoseLandmark.RIGHT_WRIST]

        LS = lm[mp_pose.PoseLandmark.LEFT_SHOULDER]
        LE = lm[mp_pose.PoseLandmark.LEFT_ELBOW]
        LW = lm[mp_pose.PoseLandmark.LEFT_WRIST]

        LH = lm[mp_pose.PoseLandmark.LEFT_HIP]
        RH = lm[mp_pose.PoseLandmark.RIGHT_HIP]

        # Prevent MediaPipe from swapping left/right if the user turns around.
        # We enforce spatial consistency based on the screen's X-axis.
        if LS.x > RS.x:
            LS, RS = RS, LS
            LE, RE = RE, LE
            LW, RW = RW, LW
            LH, RH = RH, LH

        # Helper function to convert normalized coordinates (0.0 to 1.0) to actual pixel values
        def px(p): return (int(p.x * w), int(p.y * h))

        # Calculate the internal angles of the elbows
        right_angle = calculate_angle(px(RS), px(RE), px(RW))
        left_angle = calculate_angle(px(LS), px(LE), px(LW))
        
        # --- 4. GESTURE DEFINITIONS ---
        # "Lock" gestures: Arm is raised and bent sharply (under 90 degrees)
        pose_right_lock = right_angle < 90 and RW.visibility > 0.4
        pose_left_lock = left_angle < 90 and LW.visibility > 0.4
        
        # "Hands Down" gesture: Both arms are relatively straight (>140 deg) 
        # and both wrists are positioned below their respective hips.
        pose_hands_down = (
            right_angle > 160 and left_angle > 160 and 
            RW.y > RE.y and LW.y > LE.y
        )

        # FIX 2: Prevent the active arm from randomly switching while currently locked
        if origin_locked:
            if current_active_arm == "RIGHT":
                pose_left_lock = False
            elif current_active_arm == "LEFT":
                pose_right_lock = False

        # --- 5. STATE MACHINE (HIERARCHY OF ACTIONS) ---
        # Priority 1: Safety switch. If hands go down, immediately disengage.
        if pose_hands_down:
            origin_locked = False
            current_active_arm = "NONE"
            lateral_command = "HOVER"

        # Priority 2: If we are idle, listen for an arm-lock gesture to engage.
        elif not origin_locked:
            if pose_right_lock:
                origin_locked = True
                origin_x, origin_y = px(RS)
                current_active_arm = "RIGHT"

            elif pose_left_lock:
                origin_locked = True
                origin_x, origin_y = px(LS)
                current_active_arm = "LEFT"

        # --- 6. STEERING LOGIC ---
        if origin_locked and not call_swarm:
            # deadzone1 = 0.10
            # deadzone2 = 0.20  

            # If the active arm extends past 100 degrees, trigger a lateral move
            if current_active_arm == "RIGHT":
                if right_angle > 100:
                    lateral_command = "MOVE RIGHT"

            elif current_active_arm == "LEFT":
                if left_angle > 100:
                    lateral_command = "MOVE LEFT"

        # --- 7. VISUALIZATION & HUD ---
        # Calculate and draw the center point between the hips (waist center)
        print(LS)
        print(RS)
        if LS.visibility > 0.3 and RS.visibility > 0.3:
            abs_cx = int((LS.x + RS.x) * 0.5 * w)
            abs_cy = int((LS.y + RS.y) * 0.5 * h)
            waist_center = (abs_cx - w // 2, abs_cy - h // 2)
            cv2.circle(frame_copy, (abs_cx, abs_cy), 8, (255, 0, 0), -1)

        # Draw the skeletal skeleton overlay
        mp_drawing.draw_landmarks(
            frame_copy,
            results_pose.pose_landmarks,
            mp_pose.POSE_CONNECTIONS
        )

        # Draw the Head-Up Display (HUD) text
        display_text = f"{current_active_arm} ARM ACTIVE" if origin_locked else "IDLE - RAISE ARM TO CONTROL"
        cv2.putText(frame_copy, display_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame_copy, f"Steering: {lateral_command}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    else:
        # Fallback if no person is detected in the frame
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
        lateral_command # Payload exported for ROS integration
    )

def main():
    """
    Main loop: Opens the webcam, manages persistent state variables, 
    and handles the user interface.
    """
    # Initialize persistent state variables across frames
    origin = (0, 0)
    origin_locked = False
    active_arm = "NONE" 

    # Connect to the default system webcam
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to read frame from camera.")
            break

        # Pass the current state into the frame processor, and receive the updated state
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

        # Render the processed output to the screen
        cv2.imshow("Drone Controller", processed_frame)

        # Listen for the 'ESC' or 'q' key to safely exit the application
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break

    # Clean up resources
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
