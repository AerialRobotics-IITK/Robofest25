import cv2
import mediapipe as mp
from collections import deque

# ================= MediaPipe Setup =================
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    max_num_hands=1,
    model_complexity=1,
    min_detection_confidence=0.8,
    min_tracking_confidence=0.8
)

# ================= Temporal Smoothing =================
gesture_window = deque(maxlen=10)

def stable_gesture(g):
    if g is None:
        return None
    gesture_window.append(g)
    if gesture_window.count(g) >= 7:
        return g
    return None

# ================= Finger Logic =================
def finger_states(lm):
    # Index, Middle, Ring, Pinky
    tips = [8, 12, 16, 20]
    pips = [6, 10, 14, 18]

    states = []
    for t, p in zip(tips, pips):
        states.append(lm[t].y < lm[p].y)  # True = finger up
    return states

def classify_gesture(hand_landmarks):
    lm = hand_landmarks.landmark
    fingers = finger_states(lm)
    count = sum(fingers)

    # Fist
    if count == 0:
        return "HOLD"

    # Index finger only
    if count == 1 and fingers[0]:
        return "LEFT"

    # Open palm (allow 4 or 5)
    if count >= 4:
        return "RIGHT"

    return None

# ================= FSM =================
class StateFSM:
    def __init__(self):
        self.state = "HOLD"

    def update(self, gesture):
        if gesture is None:
            self.state = "HOLD"
        else:
            self.state = gesture
        return self.state

def get_state(rgb):
    results = hands.process(rgb)
    gesture = None
    if results.multi_hand_landmarks:
        gesture = classify_gesture(results.multi_hand_landmarks[0])

    gesture = stable_gesture(gesture)
    if gesture is not None:
        return gesture
    else:
        return "HOLD"

# ================= Main Loop =================
if __name__=="__main__":
    cap = cv2.VideoCapture(0)
    fsm = StateFSM()

    print("Running gesture FSM | Press 'q' to quit")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb)

        gesture = None

        if results.multi_hand_landmarks:
            gesture = classify_gesture(results.multi_hand_landmarks[0])

        gesture = stable_gesture(gesture)
        state = fsm.update(gesture)

        # -------- UI --------
        cv2.putText(frame, f"STATE: {state}", (30, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0, 255, 0), 4)

        cv2.imshow("Easy Gesture FSM (MediaPipe)", frame)

        # Print only stable states
        if gesture is not None:
            print("STATE:", state)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    hands.close()
    cv2.destroyAllWindows()
