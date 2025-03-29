package frc.utils;

public class Wrapper {
  public static double wrapRadian(double angle) {
    angle = (angle + Math.PI) % (2 * Math.PI);
    if (angle < 0) {
      angle += (2 * Math.PI);
    }
    return angle - Math.PI;
  }
}
