import cv2
import mediapipe as mp
import numpy as np
import serial
import time

# ——— CONFIG —————————————————————————————————————————————————————————————
SERIAL_PORT = '/dev/cu.usbmodem1401'
BAUDRATE = 9600
SEND_INTERVAL = 0.1      # seconds between sends (10 Hz)
SMOOTH_FACTOR = 0.6      # 0→no smoothing, 1→freeze

# Safe bounds per servo:
#   S0–S3 : [20°, 160°], S4 (clamp) : [80°, 160°]
BOUNDS = [
    (20, 160),  # S0 base yaw
    (20, 160),  # S1 shoulder
    (60, 160),  # S2 elbow
    (20, 160),  # S3 wrist pitch
    (85, 160),  # S4 clamp
]

# ——— SETUP —————————————————————————————————————————————————————————————
ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
time.sleep(2)

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False,
                       max_num_hands=1,
                       min_detection_confidence=0.7,
                       min_tracking_confidence=0.7)
mp_draw = mp.solutions.drawing_utils
cap = cv2.VideoCapture(0)

# start in the exact midpoint of each range
prev_angles = [(mn+mx)/2 for mn, mx in BOUNDS]
last_send_time = 0.0


def clamp(v, vmin, vmax):
    return max(vmin, min(v, vmax))


def send_servo(idx, angle):
    ser.write(f"S{idx},{int(angle)}\n".encode())


# ——— MAIN LOOP —————————————————————————————————————————————————————————
try:
    while True:
        now = time.time()
        ret, frame = cap.read()
        if not ret:
            break

        # 1) run hand detection
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = hands.process(rgb)

        if res.multi_hand_landmarks:
            h = res.multi_hand_landmarks[0]
            mp_draw.draw_landmarks(frame, h, mp_hands.HAND_CONNECTIONS)

            # 2) grab key landmarks
            w = h.landmark[0]   # wrist
            i_mcp = h.landmark[5]  # index MCP
            p_mcp = h.landmark[17]  # pinky MCP
            th = h.landmark[4]   # thumb tip
            it = h.landmark[8]   # index tip

            # 3) raw normalized inputs
            r0 = np.clip(w.x,       0, 1)             # left–right
            r1 = np.clip(1 - w.y,   0, 1)             # up–down
            r2 = np.clip(w.z,      -0.2, 0.2)         # forward–back

            # 4) palm normal for pitch
            u = np.array([i_mcp.x - w.x,
                          i_mcp.y - w.y,
                          i_mcp.z - w.z])
            v = np.array([p_mcp.x - w.x,
                          p_mcp.y - w.y,
                          p_mcp.z - w.z])
            normal = np.cross(u, v)
            pitch = np.degrees(np.arctan2(normal[1], normal[2]))  # −90…90

            # 5) pinch distance
            d = np.linalg.norm([th.x - it.x,
                                th.y - it.y,
                                th.z - it.z])

            # 6) map into the full safe ranges
            raw = [
                np.interp(r0,     [0, 1],      BOUNDS[0]),  # S0
                np.interp(r1,     [0, 1],      BOUNDS[1]),  # S1
                np.interp(r2,     [-0.2, 0.2], BOUNDS[2]),  # S2
                np.interp(pitch,  [-90, 90],   BOUNDS[3]),  # S3
                np.interp(d,      [0.02, 0.15], BOUNDS[4]),  # S4
            ]

            # 7) exponential smoothing
            new_angles = []
            for i, (prev, targ) in enumerate(zip(prev_angles, raw)):
                ang = prev * SMOOTH_FACTOR + targ * (1 - SMOOTH_FACTOR)
                mn, mx = BOUNDS[i]
                ang = clamp(ang, mn, mx)
                new_angles.append(ang)
            prev_angles[:] = new_angles

            # 8) send at most at SEND_INTERVAL
            if now - last_send_time >= SEND_INTERVAL:
                for idx, ang in enumerate(new_angles):
                    send_servo(idx, ang)
                last_send_time = now

            # 9) overlay for live debugging
            for i, ang in enumerate(new_angles):
                cv2.putText(frame, f"S{i}:{int(ang):3d}",
                            (10, 30 + 20*i),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("Smoothed Hand→Servo Test", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

finally:
    cap.release()
    hands.close()
    cv2.destroyAllWindows()
    ser.close()
    print("Connection closed")
