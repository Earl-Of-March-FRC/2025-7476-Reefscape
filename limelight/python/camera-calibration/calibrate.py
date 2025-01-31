import cv2 as cv

print("starting camera...", end="\r")
cap = cv.VideoCapture(0)
num = 0

print("hit S to save an image")
print("hit D to quit")
while True:
    ret, frame = cap.read()

    k = cv.waitKey(1)
    if k == ord("d"):
        break
    elif k == ord("s"):
        success = cv.imwrite("C:\\Users\\EOM FRC\\Documents\\GitHub\\2025-7476-Reefscape\\limelight\\python\\camera-calibration\\images\\img" + str(num) + ".png", frame)
        print(f"image {num} {'saved' if success else 'failed to save (check path?)'}")
        num += 1

    cv.imshow("Camera Calibration", frame)

cap.release()
cv.destroyAllWindows()