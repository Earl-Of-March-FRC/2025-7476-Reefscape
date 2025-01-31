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
MIN_AREA = 3000  # Minimum area of the contour to be considered
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

def find_largest_algae(image):
    # Convert the input image to the HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Apply Gaussian Blur to reduce noise
    blurred = cv2.GaussianBlur(hsv, (9, 9), 0)

    # Create a mask for the ball color
    mask = cv2.inRange(blurred, LOWER_BALL, UPPER_BALL)

    # Apply morphological operations to reduce noise
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Use Canny edge detection to find edges
    edges = cv2.Canny(mask, 100, 200)
    
    # Dilate the edges to connect broken parts, especially for obstructions
    dilated_edges = cv2.dilate(edges, None, iterations=5)

    # Fill in the dilated edges (inpainting) to smooth out broken parts
    filled_edges = cv2.dilate(dilated_edges, None, iterations=2)  # further dilation to fill the edges

    # Find contours in the filled edge-detected image
    contours, _ = cv2.findContours(filled_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    largest_ball = None
    largest_area = 0

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
                # Check if this is the largest algae ball so far
                if area > largest_area:
                    largest_area = area
                    largest_ball = (x, y, radius)

    if largest_ball:
        # Draw the largest ball on the original image
        x, y, radius = largest_ball
        cv2.circle(image, (int(x), int(y)), int(radius), (255, 0, 255), 7)

    return image, mask, edges, filled_edges, largest_ball

# Open the video stream (0 for default camera or replace with a video file path)
cap = cv2.VideoCapture(1)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open video stream.")
else:
    while True:
        # Read a frame from the camera
        ret, frame = cap.read()

        if not ret:
            print("Error: Failed to capture image.")
            break
        
        # Detect the largest algae in the frame
        processed_frame, mask, edges, filled_edges, largest_ball = find_largest_algae(frame)

        # Display the processed frame
        cv2.imshow("Ball Detection", processed_frame)
        cv2.imshow("Contour", mask)
        cv2.imshow("Edges", edges)
        cv2.imshow("Filled Edges", filled_edges)

        # Check for a key press to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
            break

    # Release the video capture object and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()
