package frc.utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

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

  /*
   * 
   * public static Pose3d getRobotPose3dFromCamera(Transform3d camToTag,
   * Transform3d robotToTag) {
   * Transform3d tagToCam = camToTag.inverse();
   * 
   * Pose3d robotToTPose3d = new Pose3d(robotToTag.getX(), robotToTag.getY(),
   * robotToTag.getZ(),
   * robotToTag.getRotation());
   * 
   * return robotToTPose3d.transformBy(tagToCam);
   * }
   * 
   */

  public static Transform3d getRobotPose3dFromCamera(Transform3d camToTag, Transform3d robotToTag) {
    Transform3d tagToCam = camToTag.inverse();

    return robotToTag.plus(tagToCam);
  }

}