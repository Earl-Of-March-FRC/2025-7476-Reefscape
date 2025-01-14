import cv2
import numpy as np

# Approximate algae color in HSV
LOWER_BALL = np.array([80, 50, 60])  # Lower bound for algae ball color
UPPER_BALL = np.array([100, 255, 255])  # Upper bound for algae ball color

# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot):
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

    # Initialize variables
    algae_count = 1  # Counter for detected algae
    llpython = [0] * 8  # Initialize with 8 zeros (two bounding boxes)
    largestContour = None  # Initialize largest contour

    if contours:
        # Find the largest contour by area
        largestContour = max(contours, key=cv2.contourArea)

        # Iterate over all contours
        for contour in contours:
            # Calculate the area and circularity of the contour
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            circularity = 4 * np.pi * (area / (perimeter * perimeter)) if perimeter > 0 else 0

            # Check if the contour meets the criteria
            if area > 200 and circularity > 0.5:
                # Approximate the contour to a circle
                (x, y), radius = cv2.minEnclosingCircle(contour)

                # Only consider significant circles
                if radius > 10:
                    # Draw the circle on the original image
                    cv2.circle(image, (int(x), int(y)), int(radius), (255, 0, 255), 2)
                    cv2.putText(image, f"{algae_count}", (int(x) - 20, int(y) - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                    # Print the values of the algae's attributes
                    print(f"Circularity: {circularity}\nArea: {area}\nAlgae: {algae_count}\n")

                    # Update llpython array with bounding box values
                    x, y, w, h = cv2.boundingRect(contour)
                    if algae_count <= 2:
                        llpython[(algae_count - 1) * 4:algae_count * 4] = [x, y, w, h]

                    # Increment the algae count
                    algae_count += 1

    # Return the largest contour, the modified image, and the custom robot data
    return largestContour, image, llpython
