package frc.utils;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * Utility class for calculating transformations related to tags and cameras.
 */
public class TagUtils {

  /**
   * Calculates the transformation from the robot's coordinate system to the
   * camera's coordinate system
   * using the given transformations from the camera to a tag and from the robot
   * to the same tag.
   *
   * @param camToTag   The transformation from the camera's coordinate system to
   *                   the tag's coordinate system.
   * @param robotToTag The transformation from the robot's coordinate system to
   *                   the tag's coordinate system.
   * @return The transformation from the robot's coordinate system to the camera's
   *         coordinate system.
   */
  public static Transform3d getRobotToCam(Transform3d camToTag, Transform3d robotToTag) {
    // Invert camToTag to get tag to camera transform
    Transform3d tagToCam = camToTag.inverse();
    // Combine robotToTag and tagToCam to get robotToCam
    return robotToTag.plus(tagToCam);
  }

}