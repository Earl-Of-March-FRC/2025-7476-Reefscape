import org.opencv.core.*;
import org.opencv.highgui.HighGui;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;

public class AlgaeDetection {
    private static final Logger logger = Logger.getLogger(AlgaeDetection.class.getName());

    // Constants
    private static final double KNOWN_DIAMETER = 475.00; // mm

    private static final Scalar LOWER_BALL = new Scalar(80, 50, 60); // HSV lower-bound
    private static final Scalar UPPER_BALL = new Scalar(100, 255, 255); // HSV upper-bound

    // Contour Conditionals
    private static final int MIN_AREA = 3000;
    private static final double MIN_CIRCULARITY = 0.3;

    private static final Mat CAMERA_MATRIX = new Mat(3, 3, CvType.CV_64F);

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        CAMERA_MATRIX.put(0, 0, 1413.70008, 0, 314.24724784);
        CAMERA_MATRIX.put(1, 0, 0, 1437.31, 240.10474105);
        CAMERA_MATRIX.put(2, 0, 0, 0, 1);
    }

    private static final String STREAM_URL = "http://photonvision.local:1181/stream.mjpg";

    static class ObjectDetection {
        private Scalar lowerBound;
        private Scalar upperBound;
        private int minArea;
        private double minCircularity;

        public ObjectDetection(Scalar lowerBound, Scalar upperBound, int minArea, double minCircularity) {
            this.lowerBound = lowerBound;
            this.upperBound = upperBound;
            this.minArea = minArea;
            this.minCircularity = minCircularity;
        }

        public AlgaeResult findLargestAlgae(Mat image) {

            // Create a padded version of the image to handle partial objects near the
            // borders
            int padding = 50;
            Mat paddedImage = new Mat();
            Core.copyMakeBorder(image, paddedImage, padding, padding, padding, padding, Core.BORDER_CONSTANT,
                    new Scalar(0, 0, 0));

            // Convert to HSV
            Mat hsv = new Mat();
            Imgproc.cvtColor(paddedImage, hsv, Imgproc.COLOR_BGR2HSV);

            // Apply Gaussian Blur
            Mat blurred = new Mat();
            Imgproc.GaussianBlur(hsv, blurred, new Size(9, 9), 0);

            // Create a mask for the algae color
            Mat mask = new Mat();
            Core.inRange(blurred, lowerBound, upperBound, mask);

            // Morphological operations
            Imgproc.erode(mask, mask, new Mat(), new Point(-1, -1), 2);
            Imgproc.dilate(mask, mask, new Mat(), new Point(-1, -1), 2);

            // Canny edge detection
            Mat edges = new Mat();
            Imgproc.Canny(mask, edges, 100, 300);

            // Dilate the edges to connect broken parts
            Mat dilatedEdges = new Mat();
            Imgproc.dilate(edges, dilatedEdges, new Mat(), new Point(-1, -1), 3);

            // Further dilation to fill the edges
            Mat filledEdges = new Mat();
            Imgproc.dilate(dilatedEdges, filledEdges, new Mat(), new Point(-1, -1), 3);

            // Find contours
            java.util.List<MatOfPoint> contours = new java.util.ArrayList<>();
            Imgproc.findContours(filledEdges, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Variables to store the largest algae
            Point largestBallCenter = null;
            double largestArea = 0;
            double largestRadius = 0;

            // Iterate over the contours
            for (MatOfPoint contour : contours) {

                double area = Imgproc.contourArea(contour);

                // Convert MatOfPoint to MatOfPoint2f for arcLength
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

                double perimeter = Imgproc.arcLength(contour2f, true);
                double circularity = (perimeter > 0) ? (4 * Math.PI * area / (perimeter * perimeter)) : 0;

                if (area > minArea && circularity > minCircularity) {
                    // Approximate the contour to a circle
                    RotatedRect minEnclosingCircle = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
                    Point center = minEnclosingCircle.center;
                    double radius = minEnclosingCircle.size.height / 2;

                    if (radius > 10) {
                        // Check if this is the largest algae ball so far
                        if (area > largestArea) {
                            largestArea = area;
                            largestBallCenter = center;
                            largestRadius = radius;
                        }
                    }
                }

            }

            if (largestBallCenter != null) {
                // Unpad the cords and draw the largest ball

                int unpaddedX = (int) largestBallCenter.x - padding;
                int unpaddedY = (int) largestBallCenter.y - padding;
                Imgproc.circle(image, new Point(unpaddedX, unpaddedY), (int) largestRadius, new Scalar(255, 0, 255), 5);
            }

            return new AlgaeResult(image, largestBallCenter, largestRadius);

        }
    }

    static class Computation {
        double focalLengthX;
        double objectRealWidth;
        static Mat CAMERA_MATRIX;

        @SuppressWarnings("static-access")
        public Computation(double focalLengthX, double objectRealWidth, Mat cameraMatrix) {
            this.focalLengthX = focalLengthX;
            this.objectRealWidth = objectRealWidth;
            this.CAMERA_MATRIX = cameraMatrix; // Initialize with passed camera matrix
        }

        public double calculateDistance(double detectionWidth) {
            return ((objectRealWidth * focalLengthX) / detectionWidth) - 20; // mm
        }

        public double calculateHorizontalAngle(Mat frame, double objectCenterX, double cameraOffset) {
            try {
                double screenCenterX = frame.width() / 2;
                double screenCenterY = frame.height() / 2;

                // Adjust the object center x-coordinate based on camera offset
                objectCenterX -= cameraOffset; // offset in mm

                Mat matInverted = new Mat();
                Core.invert(CAMERA_MATRIX, matInverted);

                // Calculate vector1 and vector2
                MatOfFloat vector1 = new MatOfFloat((float) objectCenterX, (float) screenCenterY, 1.0f);
                MatOfFloat vector2 = new MatOfFloat((float) screenCenterX, (float) screenCenterY, 1.0f);

                // Convert MatOfFloat to float array
                float[] vec1Arr = vector1.toArray();
                float[] vec2Arr = vector2.toArray();

                // Perform the dot product and angle calculation
                double dotProduct = vec1Arr[0] * vec2Arr[0] + vec1Arr[1] * vec2Arr[1] + vec1Arr[2] * vec2Arr[2];

                double norm1 = Math.sqrt(vec1Arr[0] * vec1Arr[0] + vec1Arr[1] * vec1Arr[1] + vec1Arr[2] * vec1Arr[2]);
                double norm2 = Math.sqrt(vec2Arr[0] * vec2Arr[0] + vec2Arr[1] * vec2Arr[1] + vec2Arr[2] * vec2Arr[2]);

                double cosAngle = dotProduct / (norm1 * norm2);
                double realAngle = Math.toDegrees(Math.acos(cosAngle));

                if (objectCenterX < screenCenterX) {
                    realAngle *= -1;
                }

                return realAngle;

            } catch (Exception e) {
                System.out.println("Error occurred while calculating horizontal angle");
                return 0.0;
            }
        }

        public double calculateVerticalAngle(Mat frame, double objectCenterY, double cameraOffset) {
            try {
                double screenCenterX = frame.width() / 2;
                double screenCenterY = frame.height() / 2;

                // Adjust the object center y-coordinate based on camera offset
                objectCenterY -= cameraOffset; // offset in mm

                Mat matInverted = new Mat();
                Core.invert(CAMERA_MATRIX, matInverted); // Invert camera matrix

                // Calculate vector1 and vector2
                MatOfFloat vector1 = new MatOfFloat((float) screenCenterX, (float) objectCenterY, 1.0f);
                MatOfFloat vector2 = new MatOfFloat((float) screenCenterX, (float) screenCenterY, 1.0f);

                // Convert MatOfFloat to float array
                float[] vec1Arr = vector1.toArray();
                float[] vec2Arr = vector2.toArray();

                // Perform the dot product and angle calculation
                double dotProduct = vec1Arr[0] * vec2Arr[0] + vec1Arr[1] * vec2Arr[1] + vec1Arr[2] * vec2Arr[2];

                double norm1 = Math.sqrt(vec1Arr[0] * vec1Arr[0] + vec1Arr[1] * vec1Arr[1] + vec1Arr[2] * vec1Arr[2]);
                double norm2 = Math.sqrt(vec2Arr[0] * vec2Arr[0] + vec2Arr[1] * vec2Arr[1] + vec2Arr[2] * vec2Arr[2]);

                double cosAngle = dotProduct / (norm1 * norm2);
                double realAngle = Math.toDegrees(Math.acos(cosAngle));

                if (objectCenterY < screenCenterY) {
                    realAngle *= -1;
                }

                return -realAngle;

            } catch (Exception e) {
                System.out.println("Error occurred while calculating vertical angle");
                return 0.0;
            }
        }
    }

    public static void openStreamUntilSuccess(VideoCapture cap) {
        while (!cap.isOpened()) {
            if (cap.open(0)) {
                logger.info("Stream opened successfully.");
                return; // Exit the method once the stream opens successfully
            } else {
                logger.severe("Failed to open stream. Retrying...");
            }

            // Sleep for a short time before retrying, to avoid a busy loop
            try {
                Thread.sleep(1000); // 1 second wait
            } catch (InterruptedException e) {
                logger.severe("Sleep interrupted: " + e.getMessage());
            }
        }
    }

    public static void main(String[] args) {
        // Initialize VideoCapture
        VideoCapture cap = new VideoCapture();
        openStreamUntilSuccess(cap);

        // Initialize the detector and computation classes
        ObjectDetection detector = new ObjectDetection(UPPER_BALL, LOWER_BALL, MIN_AREA, MIN_CIRCULARITY);
        Computation computation = new Computation(CAMERA_MATRIX.get(0, 0)[0], KNOWN_DIAMETER, CAMERA_MATRIX);

        // Mat to hold the captured frame
        Mat frame = new Mat();

        while (true) {
            // Capture frame from the video stream
            if (cap.read(frame)) {
                // Process the frame to find the largest algae
                AlgaeResult result = detector.findLargestAlgae(frame);
                if (result != null) {
                    Mat imageWithAlgae = result.getImage();
                    Point algaeCenter = result.getCenter();
                    double algaeRadius = result.getRadius();

                    // Calculate distance and angles
                    double distance = (computation.calculateDistance(algaeRadius * 2)) / 10; // in cm
                    double x_angle = computation.calculateHorizontalAngle(frame, algaeCenter.x, 45.7);
                    double y_angle = computation.calculateVerticalAngle(frame, algaeCenter.y, 65);

                    // Optionally, display some information on the image
                    Imgproc.putText(imageWithAlgae, "Distance: " + distance + " cm", new Point(50, 50),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 0, 0), 2);
                }

                // Show the frame with algae detection
                HighGui.imshow("Frame with Algae", frame);

                // Wait for 1 ms for a key press to break the loop (0 to display indefinitely)
                if (HighGui.waitKey(1) == 27) { // 27 is the ASCII code for the ESC key
                    break;
                }
            }
        }

        // Release the capture
        cap.release();
        HighGui.destroyAllWindows(); // Close all OpenCV windows
    }
}
