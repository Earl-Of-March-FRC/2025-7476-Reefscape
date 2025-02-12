import cv2 as cv
import time
import math

"""
USER INPUT:

Let the user click the center of the circle and an outer portion of the circle.

WHAT THIS DOES:

center outside,distance, perfect circle, search for ligthest & darkest, rinse & repeat
"""

mouseX = 0
mouseY = 0
mousePressed = False

# Cal
originX = 0
originY = 0

outerX = 0
outerY = 0

def mouse_callback(event, x, y, flags, param):
    global mouseX
    global mouseY
    global mousePressed
    
    if event == cv.EVENT_LBUTTONDOWN:  # Left mouse button click
        print("I've just been clicked! Yay!")
        mousePressed = True
        mouseX = x
        mouseY = y

def lookForPoint():
    return [0, 0]

cv.namedWindow("snapshot")
cv.setMouseCallback("snapshot", mouse_callback)

capture = cv.VideoCapture(0) 
isTrue, frame = capture.read()

while True:
    if cv.waitKey(1) & 0xFF == ord('a'):
        isTrue, frame = capture.read()
        cv.imshow('snapshot', frame)
        cv.waitKey(1)

        while True:
            
            cv.imshow('snapshot', frame)
            cv.waitKey(1)

            if mousePressed == True:
                
                print(str(mouseX) + ", " + str(mouseY))
                originX = mouseX
                originY = mouseY

                mousePressed = False
                while True:
                    cv.imshow('snapshot', frame)
                    cv.waitKey(1)
                    if mousePressed == True:
                        print(str(mouseX) + ", " + str(mouseY))
                        outerX = mouseX
                        outerY = mouseY 
                        mousePressed = False    
                        radius = math.sqrt(math.pow(abs(originX-outerX ), 2)+math.pow(abs(originY-outerY ), 2))
                        cv.circle(frame, (originX, originY), int(radius), (0, 0, 255), thickness= 3)
                        cv.imshow("snapshot", frame)
                        cv.waitKey(0)
              





        