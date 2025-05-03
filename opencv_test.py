import cv2
import numpy as np

# Adjustable skin‐color range in HSV
lower = np.array([0, 48, 80], dtype="uint8")
upper = np.array([20, 255, 255], dtype="uint8")

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 1) Blur & convert
    blurred = cv2.GaussianBlur(frame, (7, 7), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # 2) Skin mask + clean up
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # 3) Find largest contour
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        hand = max(contours, key=cv2.contourArea)
        if cv2.contourArea(hand) > 1000:
            # 4) Draw convex hull + defects
            hull = cv2.convexHull(hand, returnPoints=False)
            defects = cv2.convexityDefects(hand, hull)
            cv2.drawContours(frame, [cv2.convexHull(hand)], -1, (0, 255, 0), 2)

            # 5) PCA for palm orientation
            data = hand.reshape(-1, 2).astype(np.float32)
            mean, eigenvectors = cv2.PCACompute(data, mean=None)

            # Create proper integer point coordinates
            center = (int(mean[0][0]), int(mean[0][1]))
            end_point = (int(mean[0][0] + eigenvectors[0][0] * 100),
                         int(mean[0][1] + eigenvectors[0][1] * 100))

            cv2.line(frame, center, end_point, (255, 0, 0), 2)
            cv2.circle(frame, center, 5, (255, 0, 0), -1)

    cv2.imshow('OpenCV Hand', frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
