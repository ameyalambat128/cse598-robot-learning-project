import cv2
import mediapipe as mp
import numpy as np
import csv
import time

# ── CONFIG ─────────────────────────────────────────────────────────────────────
# Toggle these ON/OFF to try different tests:
ENABLE_OVERLAY = True    # draw & print angles on the video feed
ENABLE_CSV_LOG = True    # record frame, timestamp & angles to CSV
USE_DUMMY_SERIAL = True    # replace real Serial with a logger

# If logging to CSV:
CSV_PATH = "angle_log.csv"

# Dummy Serial stub


class DummySerial:
    def write(self, data):
        print("SERIAL OUT ►", data.decode().strip())


# ── SETUP ──────────────────────────────────────────────────────────────────────
# 1) MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False,
                       max_num_hands=1,
                       min_detection_confidence=0.7,
                       min_tracking_confidence=0.7)
mp_draw = mp.solutions.drawing_utils

# 2) Video capture
cap = cv2.VideoCapture(0)

# 3) CSV writer
if ENABLE_CSV_LOG:
    csv_file = open(CSV_PATH, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["timestamp", "frame", "a0",
                        "a1", "a2", "a3", "a4", "a5"])

# 4) Serial port (or dummy)
if USE_DUMMY_SERIAL:
    ser = DummySerial()
else:
    import serial
    ser = serial.Serial('/dev/tty.usbmodem14101', 9600, timeout=1)
    time.sleep(2)

# ── HELPERS ────────────────────────────────────────────────────────────────────


def send_to_serial(angles):
    """Send 'a0,a1,...,a5\\n' over ser."""
    line = ",".join(str(int(np.clip(a, 0, 180))) for a in angles) + "\n"
    ser.write(line.encode())


def overlay_angles(frame, angles):
    """Draw joint indices + values on the top left corner."""
    for i, a in enumerate(angles):
        text = f"{i}:{int(a):3d}"
        cv2.putText(frame, text, (10, 30 + i*20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    print("Angles ▶", ["%3.0f" % a for a in angles])


def log_csv(frame_idx, angles):
    """Write timestamp, frame index, and angles to CSV."""
    timestamp = time.time()
    csv_writer.writerow([timestamp, frame_idx] + [f"{a:.1f}" for a in angles])


# ── MAIN LOOP ─────────────────────────────────────────────────────────────────
def main():
    frame_idx = 0
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # run MediaPipe
            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands.process(img_rgb)

            if results.multi_hand_landmarks:
                hand = results.multi_hand_landmarks[0]

                # 1) Draw landmarks
                mp_draw.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)

                # 2) Example angle mapping (wrist x/y → servos 0/1, rest fixed)
                wx, wy, wz = hand.landmark[0].x, hand.landmark[0].y, hand.landmark[0].z
                base_ang = np.interp(wx, [0, 1], [0, 180])
                shoulder_ang = np.interp(1-wy, [0, 1], [0, 180])
                elbow_ang = 90
                wrist_p = 90
                wrist_r = 90
                grip_ang = 90

                angles = [base_ang, shoulder_ang, elbow_ang,
                          wrist_p, wrist_r, grip_ang]

                # 3) Run tests:
                if ENABLE_OVERLAY:
                    overlay_angles(frame, angles)
                if ENABLE_CSV_LOG:
                    log_csv(frame_idx, angles)
                # 4) (Optionally) send to robot or dummy:
                send_to_serial(angles)

            # show the feed
            cv2.imshow('Hand Tracker Test', frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break

            frame_idx += 1

    finally:
        cap.release()
        hands.close()
        cv2.destroyAllWindows()
        if ENABLE_CSV_LOG:
            csv_file.close()


if __name__ == "__main__":
    main()
