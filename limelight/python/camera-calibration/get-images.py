import glob
import cv2 as cv
import numpy as np
from numpy.typing import NDArray

MatLike = NDArray[np.uint8]

# Checkerboard dimensions: 9x6 (9 columns, 6 rows)
board_width = 9
board_height = 6
square_size = 25  # You can adjust this to the real square size (in mm or cm)

# Termination criteria for corner refinement
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points (3D points in real-world space)
objp = np.zeros((board_width * board_height, 3), np.float32)
objp[:, :2] = np.mgrid[0:board_width, 0:board_height].T.reshape(-1, 2)

# Arrays to store object points and image points from all images
objpoints = []  # 3D points in real-world space
imgpoints = []  # 2D points in image plane

# Load all the images containing the checkerboard pattern
images = glob.glob(r"C:\Users\fence\Desktop\FRC\2025-7476-Reefscape\limelight\python\camera-calibration\images\*.png")  # Adjust the path

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    
    # Find the chessboard corners
    success, corners = cv.findChessboardCorners(gray, (board_width, board_height), None)

    if not success:
        print(f"Could not find corners in {fname}")
        continue

    # Refine the corner positions
    corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    
    # Append object points and image points
    objpoints.append(objp)
    imgpoints.append(corners2)

    # Draw and display the corners
    cv.drawChessboardCorners(img, (board_width, board_height), corners2, success)
    cv.imshow('Chessboard', img)
    # cv.waitKey(500)

cv.destroyAllWindows()

# Camera calibration to get the camera matrix and distortion coefficients
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

if ret:
    print("Camera calibration successful!")
    print("Camera Matrix (Intrinsic parameters):")
    print(mtx)
    print("Distortion Coefficients:")
    print(dist)
else:
    print("Camera calibration failed.")
