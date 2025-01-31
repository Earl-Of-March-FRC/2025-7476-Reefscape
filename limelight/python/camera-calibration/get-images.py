
import glob
import cv2 as cv
import numpy as np
from numpy.typing import NDArray

MatLike = NDArray[np.uint8]

# termination criteria
internal_points = (5, 8)
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((internal_points[0] * internal_points[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:internal_points[0], 0:internal_points[1]].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob("C:\\Users\\EOM FRC\\Documents\\GitHub\\2025-7476-Reefscape\\limelight\\python\\camera-calibration\\images\\*.png")

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    success, corners = cv.findChessboardCorners(gray, internal_points, None)

    # If found, add object points, image points (after refining them)
    if not success:
        continue

    objpoints.append(objp)

    corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    imgpoints.append(corners2)

    # Draw and display the corners
    cv.drawChessboardCorners(img, (7, 6), corners2, success)

cv.destroyAllWindows()

matrix: MatLike = cv.calibrateCamera(objpoints, imgpoints, (640, 480), None, None)[1]

print("in the following order: fx, fy, cx, cy", end="\n\n")

print(matrix[0, 0])
print(matrix[1, 1])
print(matrix[0, 2])
print(matrix[1, 2])