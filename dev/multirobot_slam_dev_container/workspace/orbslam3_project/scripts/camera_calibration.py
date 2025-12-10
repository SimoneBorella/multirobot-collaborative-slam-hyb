import cv2
import numpy as np
import glob
import time

CHECKERBOARD = (7, 6)

objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

cap = cv2.VideoCapture(0)

count = 0

while count < 20:
    ret, frame = cap.read()

    if not ret or frame is None:
        print("Failed to grab frame")
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

        cv2.drawChessboardCorners(frame, CHECKERBOARD, corners, ret)

    cv2.imshow('Calibration', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    time.sleep(0.5)

    count += 1
    print(f"Captured frame {count}")

cap.release()
cv2.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("Camera matrix:\n", mtx)
print("Distortion coefficients:\n", dist)

np.savez("./calibration/params.npz", mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)

with open("./calibration/params.txt", "w") as f:
    f.write("Camera Matrix:\n")
    f.write(np.array2string(mtx, separator=', ') + "\n\n")

    f.write("Distortion Coefficients:\n")
    f.write(np.array2string(dist, separator=', ') + "\n\n")

    f.write("Rotation Vectors:\n")
    for i, rvec in enumerate(rvecs):
        f.write(f"Image {i+1}:\n{np.array2string(rvec, separator=', ')}\n")

    f.write("\nTranslation Vectors:\n")
    for i, tvec in enumerate(tvecs):
        f.write(f"Image {i+1}:\n{np.array2string(tvec, separator=', ')}\n")
