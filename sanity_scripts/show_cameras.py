import cv2
import numpy as np

# Open both cameras
cap0 = cv2.VideoCapture(0)
cap2 = cv2.VideoCapture(2)

# Set resolution (optional)
cap0.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap0.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("Press 'q' to quit")
print(f"Camera 0 resolution: {cap0.get(cv2.CAP_PROP_FRAME_WIDTH)}x{cap0.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
print(f"Camera 2 resolution: {cap2.get(cv2.CAP_PROP_FRAME_WIDTH)}x{cap2.get(cv2.CAP_PROP_FRAME_HEIGHT)}")

while True:
    # Read frames
    ret0, frame0 = cap0.read()
    ret2, frame2 = cap2.read()

    if not ret0 or not ret2:
        print("Error reading from cameras")
        break

    # Resize frames to same height for side-by-side display
    height = 480
    frame0_resized = cv2.resize(frame0, (int(frame0.shape[1] * height / frame0.shape[0]), height))
    frame2_resized = cv2.resize(frame2, (int(frame2.shape[1] * height / frame2.shape[0]), height))

    # Combine side by side
    combined = np.hstack([frame0_resized, frame2_resized])

    # Add labels
    cv2.putText(combined, "Camera wide", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(combined, "Camera narrow", (frame0_resized.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Display
    cv2.imshow('Cameras - Press q to quit', combined)

    # Quit on 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap0.release()
cap2.release()
cv2.destroyAllWindows()
print("Cameras closed")