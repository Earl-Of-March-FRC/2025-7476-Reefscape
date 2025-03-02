import cv2
import numpy as np
import math

KNOWN_DISTANCE = 95.25  # Known distance for algae ball (in cm)
KNOWN_DIAMETER = 41.275  # Known diameter of algae ball (in cm)
FOCAL_LENGTH = 54.2193717956543 # in cm

# Approximate algae color in HSV
LOWER_BALL = np.array([80, 50, 60])  # Lower bound for algae ball color
UPPER_BALL = np.array([100, 255, 255])  # Upper bound for algae ball color

# Define minimum area and circularity for the contour filtering
MIN_AREA = 5000  # Minimum area of the contour to be considered
MIN_CIRCULARITY = 0.3  # Minimum circularity to consider as a valid algae ball

hasBorderBoxes = True


# Focal length finder function (calculates the focal length based on a reference image and real-world object size)
def focal_length_finder(measured_distance, real_diameter, radius_in_rf_image):
    # Calculate the focal length using the known reference image data
    focal_length = (radius_in_rf_image * measured_distance) / real_diameter
    return focal_length

def calculate_distance(focal_length, radius_in_image):
    # Using the formula: distance = (known diameter * focal length) / image radius
    distance = (KNOWN_DIAMETER * focal_length) / radius_in_image
    return distance


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
                x, y,_,_ = cv2.boundingRect(contour)
                algae_balls.append((x, y, radius))
                
    return algae_balls


def runPipeline(image, llrobot):
    global hasBorderBoxes
    llrobot = [0,0,0]
    hasBorderBoxes = llrobot[0] != 1
    algae_balls = find_algae(image)

    llpython = [0] * 8

    for algae_ball in algae_balls:
        x, y, radius = algae_ball
        print(radius)
    
        fl = focal_length_finder(KNOWN_DISTANCE,KNOWN_DIAMETER,radius)
        print(f"FL : {fl}")
        # Use the first detected algae ball for calculations
        distance = calculate_distance(FOCAL_LENGTH, radius)
        print(f"Distance: {distance:.2f} cm")
        
        llpython = [distance]

    largest_contour = np.array([[]])  # Placeholder, not used but required for return

    return largest_contour, image, llpython
