import org.opencv.core.*;

public class AlgaeResult {
    private Mat image;
    private Point center;
    private double radius;

    public AlgaeResult(Mat image, Point center, double radius) {
        this.image = image;
        this.center = center;
        this.radius = radius;
    }

    public Mat getImage() {
        return image;
    }

    public Point getCenter() {
        return center;
    }

    public double getRadius() {
        return radius;
    }

    public static void main(String[] args) {
        System.out.println("Loading OpenCV library: " + Core.NATIVE_LIBRARY_NAME);
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

    }
}
