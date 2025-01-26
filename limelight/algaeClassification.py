import cv2
import numpy as np

# Function to detect the algae game object in an image
def detect_algae(image):
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Approximate algae color in HSV -- might need some playing around with in person 
    lower_ball = np.array([80, 50, 60])   # Lower bound for algae ball color
    upper_ball = np.array([100, 255, 255]) # Upper bound for algae ball color

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
        if area > 1_000 and circularity > 0.6:
            # Approximate the contour to a circle
            ((x, y), radius) = cv2.minEnclosingCircle(contour)
            print(f"Radius: {radius}")
            if radius > 10:
                # Draw the circle on the original image
                cv2.circle(image, (int(x), int(y)), int(radius), (255, 0, 255), 2)
                cv2.putText(image, f"{algae_count}", (int(x) - 20, int(y) - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                algae_count += 1
                quit()

    return image, mask

# Open the video stream (0 for default camera or replace with a video file path)
cap = cv2.VideoCapture(0)

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
        
        # Detect algae in the frame
        processed_frame, mask = detect_algae(frame)

        # Display the processed frame
        cv2.imshow("Ball Detection", processed_frame)
        cv2.imshow("Mask", mask)

        # Check for a key press to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
            break

    # Release the video capture object and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()