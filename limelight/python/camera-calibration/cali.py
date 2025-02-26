import cv2 as cv
import numpy as np
import glob

# Load one test image
test_image_path = r"C:\Users\fence\Desktop\FRC\2025-7476-Reefscape\limelight\python\camera-calibration\images\img0.png"
img = cv.imread(test_image_path)
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

# Try different values for internal corner detection
best_internal_points = None

for width in range(3, 10):  # Try widths from 3 to 10
    for height in range(3, 10):  # Try heights from 3 to 10
        success, corners = cv.findChessboardCorners(gray, (width, height), None)
        
        if success:
            best_internal_points = (width, height)
            print(f"✔ Chessboard detected with internal corners: {best_internal_points}")

# Display detected corners
if best_internal_points:
    print(f"\n✅ Best internal points: {best_internal_points}")
    success, corners = cv.findChessboardCorners(gray, best_internal_points, None)
    cv.drawChessboardCorners(img, best_internal_points, corners, success)
    cv.imshow("Detected Corners", img)
    cv.waitKey(0)
    cv.destroyAllWindows()
else:
    print("\n❌ No valid internal corners found! Try adjusting the range.")

