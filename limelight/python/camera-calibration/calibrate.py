import cv2
import numpy as np

# Global variables
frozen = False
points = []  # List to store multiple points of the polygon
frozen_frame = None
STREAM_URL = "http://localhost:1181/stream.mjpg"
polygon_pixels = {}  # Dictionary to store HSV bounds for polygons
frame_counter = 0  # Frame identifier for the hashmap
saved_polygons = []  # List to store saved polygons
redo_mode = False  # Flag to allow redrawing polygons

# Mouse callback function to capture points for the polygon
def click_event(event, x, y, flags, param):
    global points, redo_mode

    if frozen and redo_mode:  # Only allow clicks when the camera is frozen and in redo mode
        if event == cv2.EVENT_LBUTTONDOWN:
            points.append((x, y))
            print("Point added:", (x, y))

# Function to get the upper and lower bounds of the pixels in HSV color space within the polygon
def get_hsv_bounds(frame, polygon_points):
    # Convert the frame to HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask for the polygon
    mask = np.zeros(hsv_frame.shape[:2], dtype=np.uint8)
    cv2.fillPoly(mask, [np.array(polygon_points, dtype=np.int32)], 255)

    # Apply the mask to get the region inside the polygon
    masked_hsv = cv2.bitwise_and(hsv_frame, hsv_frame, mask=mask)

    # Find the non-zero pixels inside the mask (the pixels that are part of the polygon)
    non_zero_pixels = masked_hsv[masked_hsv[:, :, 0] != 0]

    if non_zero_pixels.size == 0:
        return None, None  # Return None if no pixels found

    # Calculate the upper and lower bounds of the pixels (min and max values in HSV)
    lower_bound = np.min(non_zero_pixels, axis=0)
    upper_bound = np.max(non_zero_pixels, axis=0)

    # Return lower and upper bounds in the HSV space
    return lower_bound, upper_bound

# Function to calculate the common bounds across all polygons
def calculate_common_range(saved_polygons):
    # Initialize variables to track the common lower and upper bounds
    common_lower = np.array([0, 0, 0], dtype=np.float32)
    common_upper = np.array([179, 255, 255], dtype=np.float32)

    # Iterate over all saved polygons and find the intersection of the bounds
    for polygon in saved_polygons:
        upper_bound = np.array(polygon['upper'], dtype=np.float32)
        lower_bound = np.array(polygon['lower'], dtype=np.float32)

        # Update the common lower and upper bounds by finding the max lower bound and min upper bound
        common_lower = np.maximum(common_lower, lower_bound)
        common_upper = np.minimum(common_upper, upper_bound)

    # Return the common lower and upper bounds
    return common_lower, common_upper

# Open the camera
cap = cv2.VideoCapture(STREAM_URL)

cv2.namedWindow("Camera")
cv2.setMouseCallback("Camera", click_event)

while True:
    ret, frame = cap.read()

    if not ret:
        break

    if frozen:
        # If frozen, display the frozen frame and allow clicks
        display_frame = frozen_frame.copy()

        # Draw polygon if points are selected
        if len(points) > 1:
            cv2.polylines(display_frame, [np.array(points, dtype=np.int32)], isClosed=True, color=(0, 255, 0), thickness=2)

            # Get the upper and lower bounds of the pixels in HSV space
            lower_bound, upper_bound = get_hsv_bounds(frozen_frame, points)

            if lower_bound is not None and upper_bound is not None:
                # Store the bounds in the dictionary with the current frame's unique identifier
                polygon_pixels[frame_counter] = [upper_bound.tolist(), lower_bound.tolist()]

        # Instructions for the user
        cv2.putText(display_frame, "Frozen. Click multiple points to form a polygon.", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        if redo_mode:
            cv2.putText(display_frame, "Redo Mode. Press 's' to save the polygon, 'x' to stop.", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
    else:
        # Show the live feed if not frozen
        display_frame = frame.copy()
        cv2.putText(display_frame, "Press 'a' to freeze", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    # Show the camera feed
    cv2.imshow("Camera", display_frame)

    key = cv2.waitKey(1) & 0xFF

    # If 'a' is pressed, freeze the frame
    if key == ord('a'):
        frozen = not frozen
        if frozen:
            frozen_frame = frame.copy()  # Capture the current frame to freeze
            frame_counter += 1  # Increment frame counter for a new unique frame identifier
            print(f"Camera frozen! Frame {frame_counter}")
            points = []  # Reset the points for a new polygon
            redo_mode = True  # Start redo mode
        else:
            print("Camera resumed!")

    # If 's' is pressed, save the current polygon and unfreeze
    if key == ord('s') and redo_mode:
        if len(points) > 2:  # Need at least three points for a polygon
            lower_bound, upper_bound = get_hsv_bounds(frozen_frame, points)
            if lower_bound is not None and upper_bound is not None:
                # Save the polygon and bounds
                saved_polygons.append({'points': points, 'upper': upper_bound, 'lower': lower_bound})
                print(f"Polygon saved with bounds: Upper HSV: {upper_bound}, Lower HSV: {lower_bound}")
        # Reset the points and enter redo mode
        points = []
        frozen = False
        redo_mode = True  # Allow redo mode to continue for drawing new polygons
        print("Polygon saved and camera resumed. Now click multiple new points to draw a new polygon.")

    # If 'x' is pressed, exit the loop
    if key == ord('x'):
        break

# Release the capture and close any open windows
cap.release()
cv2.destroyAllWindows()

# Calculate and print the common HSV range for all saved polygons
if saved_polygons:
    common_lower, common_upper = calculate_common_range(saved_polygons)
    print(f"Common Range of HSV:\nLower: {common_lower}\nUpper: {common_upper}")

    # Calculate and print the average of the common bounds
    average_lower = np.mean(common_lower)
    average_upper = np.mean(common_upper)

    print(f"Average Common Range:\nLower: {average_lower}\nUpper: {average_upper}")
else:
    print("No polygons saved.")
