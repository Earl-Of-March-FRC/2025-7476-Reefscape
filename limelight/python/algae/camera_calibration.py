import cv2 as cv
import time

"""
USER INPUT:

Let the user click the center of the circle and an outer portion of the circle.

WHAT THIS DOES:

center outside,distance, perfect circle, search for ligthest & darkest, rinse & repeat
"""

mouseX = 0
mouseY = 0
mousePressed = False

def mouse_callback(event, x, y, flags, param):
    global mouseX
    global mouseY
    global mousePressed
    
    if event == cv.EVENT_LBUTTONDOWN:  # Left mouse button click
        print("I've just been clicked! Yay!")
        mousePressed = True
        mouseX = x
        mouseY = y

cv.namedWindow("snapshot")
cv.setMouseCallback("snapshot", mouse_callback)

capture = cv.VideoCapture(0) 
isTrue, frame = capture.read()
#cv.imshow('snapshot', frame)
while True:
    if cv.waitKey(1) & 0xFF == ord('a'):
        isTrue, frame = capture.read()
        cv.imshow('snapshot', frame)
        print("I've (allegedly) updated the frame!")
        while True:
            print("hi")
            if mousePressed == True:
                print(str(mouseX) + ", " + str(mouseY))
print("broken out?")
#while True:
 #   if mousePressed == True:
  #      print(str(mouseX) + ", " + str(mouseY))





        