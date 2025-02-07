import cv2
import numpy as np
import math
import logging

from python.network_tables import NetworkTable

KNOWN_DIAMETER = 475.00  # Known diameter of algae ball (in mm)


# Approximate algae color in HSV
LOWER_BALL = np.array([80, 50, 60])  # Lower bound for algae ball color
UPPER_BALL = np.array([100, 255, 255])  # Upper bound for algae ball color


# Define minimum area and circularity for the contour filtering
MIN_AREA = 3000  # Minimum area of the contour to be considered
MIN_CIRCULARITY = 0.3  # Minimum circularity to consider as a valid algae ball

# Define camera matrix for webcam
CAM_MATRIX = np.array([[1413.70008, 0, 314.24724784], 
                       [0, 1437.31, 240.10474105], 
                       [0, 0, 1]])

STREAM_URL = "http://localhost:1181/stream.mjpg"


# Class for Object Detection
class ObjectDetection:
    def __init__(self, lower_bound, upper_bound, min_area, min_circularity):
        # Encapsulated variables with private attributes
        self._lower_bound = lower_bound
        self._upper_bound = upper_bound
        self._min_area = min_area
        self._min_circularity = min_circularity

    # Getter methods
    @property
    def get_lower_bound(self):
        return self._lower_bound

    @property
    def get_upper_bound(self):
        return self._upper_bound

    @property
    def get_min_area(self):
        return self._min_area

    @property
    def get_min_circularity(self):
        return self._min_circularity

    def find_largest_algae(self, image):
        # Create a padded version of the image to handle partial objects near the borders
        padding = 50
        padded_image = cv2.copyMakeBorder(image, padding, padding, padding, padding, cv2.BORDER_CONSTANT, value=(0, 0, 0))

        # Convert the padded image to HSV
        hsv = cv2.cvtColor(padded_image, cv2.COLOR_BGR2HSV)

        # Apply Gaussian Blur to reduce noise
        blurred = cv2.GaussianBlur(hsv, (9, 9), 0)

        # Create a mask for the ball color
        mask = cv2.inRange(blurred, self._lower_bound, self._upper_bound)

        # Apply morphological operations to reduce noise
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Use Canny edge detection to find edges
        edges = cv2.Canny(mask, 100, 300)
        
        # Dilate the edges to connect broken parts, especially for obstructions
        dilated_edges = cv2.dilate(edges, None, iterations=3)

        # Fill in the dilated edges (inpainting) to smooth out broken parts
        filled_edges = cv2.dilate(dilated_edges, None, iterations=3)  # further dilation to fill the edges

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
            if area > self._min_area and circularity > self._min_circularity:
                # Approximate the contour to a circle
                (x, y), radius = cv2.minEnclosingCircle(contour)

                # Only consider significant circles
                if radius > 10:
                    # Check if this is the largest algae ball so far
                    if area > largest_area:
                        largest_area = area
                        largest_ball = (x, y, radius, radius*2) # returns diameter or obj width

        if largest_ball:
            # Unpad the coordinates and draw the largest ball on the original image
            x, y, radius, _ = largest_ball
            # Remove padding from coordinates (accounting for the added border)
            unpadded_x = int(x) - padding
            unpadded_y = int(y) - padding
            cv2.circle(image, (unpadded_x, unpadded_y), int(radius), (255, 0, 255), 5)

        return image, mask, edges, filled_edges, largest_ball



class Computation:
    def __init__(self, focal_length_x, object_real_width, focal_length_y=None):
        # Private variables
        self._focal_length_x = focal_length_x
        self._object_real_width = object_real_width
        self._focal_length_y = focal_length_y if focal_length_y else focal_length_x  # Assume same for y if not provided

    # Getter and setter for focal_length_x
    @property
    def focal_length_x(self):
        return self._focal_length_x

    @focal_length_x.setter
    def focal_length_x(self, value):
        if value > 0:  # Add validation if needed
            self._focal_length_x = value
        else:
            raise ValueError("Focal length must be positive")

    # Getter and setter for object_real_width
    @property
    def object_real_width(self):
        return self._object_real_width

    @object_real_width.setter
    def object_real_width(self, value):
        if value > 0:  # Add validation if needed
            self._object_real_width = value
        else:
            raise ValueError("Object real width must be positive")

    # Getter and setter for focal_length_y
    @property
    def focal_length_y(self):
        return self._focal_length_y

    @focal_length_y.setter
    def focal_length_y(self, value):
        if value > 0:  # Add validation if needed
            self._focal_length_y = value
        else:
            raise ValueError("Focal length must be positive")

    def calculate_distance_with_offset(self, detection_width):
        return ((self._object_real_width * self._focal_length_x) / detection_width) - 20  # returns in mm

    def calculate_horizontal_angle(self, frame, object_center_x, camera_offset):
        # Calculate the horizontal angle between the camera and the object.
        try:
            screen_center_x = frame.shape[1] / 2
            screen_center_y = frame.shape[0] / 2

            # Adjust the object center x-coordinate based on camera offset
            object_center_x -= camera_offset  # offset in mm 

            mat_inverted = np.linalg.inv(CAM_MATRIX)  # Invert camera matrix
            vector1 = mat_inverted.dot((object_center_x, screen_center_y, 1.0))  # Calculate vector 1
            vector2 = mat_inverted.dot((screen_center_x, screen_center_y, 1.0))  # Calculate vector 2

            # Handle division by zero when both vectors are zero vectors (will not crash)
            if np.all(vector1 == 0) and np.all(vector2 == 0):
                return 0.0

            cos_angle = vector1.dot(vector2) / (np.linalg.norm(vector1) * np.linalg.norm(vector2))
            real_angle = math.degrees(math.acos(cos_angle))

            if object_center_x < screen_center_x:
                real_angle *= -1

            return real_angle

        except Exception as e:
            logging.error("Error occurred while calculating horizontal angle: %s", e)
            return 0.0

    def calculate_vertical_angle(self, frame, object_center_y, camera_offset):
        # Calculate the vertical angle between the camera and the object.
        try:
            screen_center_x = frame.shape[1] / 2
            screen_center_y = frame.shape[0] / 2

            # Adjust the object center y-coordinate based on camera offset
            object_center_y -= camera_offset  # offset in mm 

            mat_inverted = np.linalg.inv(CAM_MATRIX)  # Invert camera matrix
            vector1 = mat_inverted.dot((screen_center_x, object_center_y, 1.0))  # Calculate vector 1
            vector2 = mat_inverted.dot((screen_center_x, screen_center_y, 1.0))  # Calculate vector 2

            # Handle division by zero when both vectors are zero vectors (will not crash)
            if np.all(vector1 == 0) and np.all(vector2 == 0):
                return 0.0

            cos_angle = vector1.dot(vector2) / (np.linalg.norm(vector1) * np.linalg.norm(vector2))
            real_angle = math.degrees(math.acos(cos_angle))

            if object_center_y < screen_center_y:
                real_angle *= -1

            return -real_angle

        except Exception as e:
            logging.error("Error occurred while calculating vertical angle: %s", e)
            return 0.0


# Main part of the code
def main():
    
    # Try to open the video stream and keep trying until it succeeds
    cap = None
    while cap is None or not cap.isOpened():
        cap = cv2.VideoCapture(STREAM_URL)
        if not cap.isOpened():
            logging.error("Error: Could not open video stream. Retrying...")
            cv2.waitKey(1000)  # Wait 1 second before retrying


    # Initialize ObjectDetection and Computation classes
    obj_detection = ObjectDetection(LOWER_BALL, UPPER_BALL, MIN_AREA, MIN_CIRCULARITY)
    computation = Computation(CAM_MATRIX[0][0], KNOWN_DIAMETER,CAM_MATRIX[1][1]) # Diameter in mm 
    network_table = NetworkTable()

    while True:
        # Read a frame from the camera
        ret, frame = cap.read()

        if not ret:
            logging.error("Error: Failed to capture image.")
            break
        
        # Detect the largest algae in the frame
        processed_frame, mask, edges, filled_edges, largest_ball = obj_detection.find_largest_algae(frame)

        if largest_ball:
            # If we found a valid ball, calculate distance and display it
            x, y, radius,diameter = largest_ball
            distance = (computation.calculate_distance_with_offset(diameter)) / 10 # in cm
            x_angle = computation.calculate_horizontal_angle(processed_frame, x, 45.7)
            y_angle = computation.calculate_vertical_angle(processed_frame, y, 65)
            # distance_in_inches = distance / 25.4

            network_table.send_data(distance, x_angle, y_angle)

            # print(f"Distance to algae ball: {distance:.2f} cm")
            # print(f"Angle to algae ball relative to camera: {angle:.2f} deg")

        # Display the processed frame
        cv2.imshow("Ball Detection", processed_frame)
        cv2.imshow("Contour", mask)
        # cv2.imshow("Edges", edges)
        # cv2.imshow("Filled Edges", filled_edges)

        # Check for a key press to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
            break

    # Release the video capture object and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()