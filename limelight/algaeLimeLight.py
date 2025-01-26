import cv2
import numpy as np

# Approximate algae color in HSV
LOWER_BALL = np.array([80, 50, 60])  # Lower bound for algae ball color
UPPER_BALL = np.array([100, 255, 255])  # Upper bound for algae ball color


# Define minimum area and circularity for the contour filtering
MIN_AREA = 5000  # Minimum area of the contour to be considered
MIN_CIRCULARITY = 0.3  # Minimum circularity to consider as a valid algae ball

def find_algae(image):
    # Convert the input image to the HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Apply Gaussian Blur to reduce noise
    blurred = cv2.GaussianBlur(hsv, (9, 9), 0)

    # Create a mask for the ball color
    mask = cv2.inRange(blurred, LOWER_BALL, UPPER_BALL)

    # Apply morphological operations to reduce noise
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    algae_balls = []

    # Iterate over all contours
    for contour in contours:
        # Calculate the area and circularity of the contour
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        circularity = 4 * np.pi * (area / (perimeter * perimeter)) if perimeter > 0 else 0

        # Check if the contour meets the criteria
        if area > MIN_AREA and circularity > MIN_CIRCULARITY:
            # Approximate the contour to a circle
            (x, y), radius = cv2.minEnclosingCircle(contour)

            # Only consider significant circles
            if radius > 10:
                if hasBorderBoxes:
                    # Draw the circle on the original image
                    cv2.circle(image, (int(x), int(y)), int(radius), (255, 0, 255), 7)
                    

                # Print the values of the algae's attributes
                print(f"Circularity: {circularity}\nArea: {area}\n")

                # Update llpython array with bounding box values
                x, y, w, h = cv2.boundingRect(contour)
                algae_balls.append((x, y, w, h))
                
    return algae_balls
    
def runPipeline(image, llrobot):
    global hasBorderBoxes
    hasBorderBoxes = llrobot[0] == 1
    print(hasBorderBoxes)
    algae_balls = find_algae(image)
    
    llpython = [0] * 8

    if algae_balls:
        x1, y1, w1, h1 = algae_balls[0]
        x2, y2, w2, h2 = algae_balls[0]
        llpython = [x1, y1, w1, h1,x2,y2,w2,h2]
    
    largest_contour = np.array([[]])  # Placeholder, not used but required for return

    # Return the largest contour, the modified image, and the custom robot data
    return largest_contour, image, llpython