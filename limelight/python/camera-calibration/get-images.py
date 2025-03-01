import cv2
import numpy as np

# Function to detect the algae game object in an image
def detect_algae(image):
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Approximate algae color in HSV -- might need some playing around with in person 
    lower_ball = np.array([80, 50, 60])     # Lower bound for algae ball color
    upper_ball = np.array([100, 255, 255])
 # Upper bound for algae ball color

    # Apply Gaussian Blur to reduce noise
    blurred = cv2.GaussianBlur(hsv, (9, 9), 0)

    # Create a mask for the ball color
    mask = cv2.inRange(blurred, lower_ball, upper_ball)

    # Apply morphological operations to reduce noise
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    algae_count = 1  # Initialize the algae count
    for contour in contours:

        # Calculate the area and circularity of the largest contour
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        circularity = 4 * np.pi * (area / (perimeter * perimeter)) if perimeter > 0 else 0
        print(f"Circularity: {circularity}\nArea: {area}\nAlgae: {algae_count}\n")
        # Ignore small contours or those that are not circular
        if area > 500 and circularity > 0.3:
            # Approximate the contour to a circle
            ((x, y), radius) = cv2.minEnclosingCircle(contour)

            if radius > 10:
                # Draw the circle on the original image
                cv2.circle(image, (int(x), int(y)), int(radius), (255, 0, 255), 2)
                cv2.putText(image, f"{algae_count}", (int(x) - 20, int(y) - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                algae_count += 1

    return image, mask

# Load the image
image_path = r"C:\Users\fence\Desktop\FRC\2025-7476-Reefscape\limelight\python\ball-imgs\images (4).jpg" # change img (#) to test different images
image = cv2.imread(image_path)

# Check if image was successfully loaded
if image is None:
    print("Error: Could not load image.")
else:
    # Detect the ball in the image
    processed_image, mask = detect_algae(image)

    # Display the processed image
    cv2.imshow("Ball Detection", processed_image)
    cv2.imshow("Mask", mask)

    # Wait until a key is pressed, then close the windows
    cv2.waitKey(0)
    cv2.destroyAllWindows()
