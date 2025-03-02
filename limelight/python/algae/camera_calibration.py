import cv2 as cv
import math

"""
USER INPUT:

Let the user click the center of the circle and an outer portion of the circle.

WHAT THIS DOES:

center outside,distance, perfect circle, search for ligthest & darkest, rinse & repeat
"""

mouseX: int = 0
mouseY: int = 0
mousePressed: bool = False

# Cal
originX: int = 0
originY: int = 0

outerX: int = 0
outerY: int = 0

def mouse_callback(event, x, y, flags, param) -> None:
    global mouseX
    global mouseY
    global mousePressed
    
    if event == cv.EVENT_LBUTTONDOWN:  # Left mouse button click (down)
        print("Mouse has been clicked: " + str(mouseX) + ", " + str(mouseY))
        mousePressed = True
        mouseX = x
        mouseY = y
        # print("Mouse clicked? " + str(mousePressed))

cv.namedWindow("snapshot")
cv.setMouseCallback("snapshot", mouse_callback)

capture = cv.VideoCapture(0) 
isTrue, frame = capture.read()

def calibrate() -> None:
    global mouseX
    global mouseY
    global mousePressed
    global originX
    global originY
    global outerX
    global outerY
    global capture
    global isTrue, frame

    # check if the key pressed was "a",
    if cv.waitKey(0) & 0xFF == ord('a'):
        # update the frame
        isTrue, frame = capture.read()
        cv.imshow('snapshot', frame)
        cv.waitKey(1) # 

        # make sure that mouse clicks have no effect until a snapshot has been taken (avoid unexpected behaviour)
        mousePressed = False

        while mousePressed == False:
            cv.imshow('snapshot', frame)
            cv.waitKey(1)

            if mousePressed:
                originX = mouseX
                originY = mouseY
                print("Origin coordinates: (" + str(originX) + ", " + str(originY) + ")")

                mousePressed = False
                while mousePressed == False:
                    cv.imshow('snapshot', frame)
                    cv.waitKey(1)
                    if mousePressed:
                        outerX = mouseX
                        outerY = mouseY 
                        print("Outer coordinates: (" + str(originX) + ", " + str(originY) + ")")
   
                        mousePressed = False
                        radius = math.sqrt(math.pow(abs(originX-outerX ), 2)+math.pow(abs(originY-outerY ), 2))
                        cv.circle(frame, (originX, originY), int(radius), (0, 0, 255), thickness= 3)
                        cv.imshow("snapshot", frame)
                        cv.waitKey(0)
                        break
                break

def main() -> None:
    while True: 
        calibrate()
        print("Done calibrating! Going for another!")

if __name__ == "__main__":
    main()


        