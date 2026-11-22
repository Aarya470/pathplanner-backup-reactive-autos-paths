package com.pathplanner.lib.detection;

import com.pathplanner.lib.obstacle.AllianceColor;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * One robot seen by a {@link RobotDetector}, measured from your robot.
 *
 * <p>Distance and bearings are worked out against the pose your robot had when this object was
 * made, so read them right away instead of keeping the object for later loops.
 */
public class DetectedRobot {
  private final Translation2d fieldPosition;
  private final AllianceColor alliance;
  private final double timestampSeconds;
  private final double distanceMeters;
  private final double bearingDegrees;
  private final double cameraBearingDegrees;

  /**
   * @param fieldPosition where the other robot is on the field, in meters (blue origin)
   * @param alliance its bumper color, or UNKNOWN if the model does not say
   * @param timestampSeconds when the camera saw it (FPGA time)
   * @param robotPose your robot's pose
   * @param cameraYawRadians which way the camera points, from your robot's front (0 = forward,
   *     PI = backward, counterclockwise positive)
   */
  public DetectedRobot(
      Translation2d fieldPosition,
      AllianceColor alliance,
      double timestampSeconds,
      Pose2d robotPose,
      double cameraYawRadians) {
    this.fieldPosition = fieldPosition;
    this.alliance = alliance;
    this.timestampSeconds = timestampSeconds;

    Translation2d offset = fieldPosition.minus(robotPose.getTranslation());
    this.distanceMeters = offset.getNorm();
    Rotation2d fieldAngle = offset.getAngle();
    Rotation2d fromFront = fieldAngle.minus(robotPose.getRotation());
    this.bearingDegrees = fromFront.getDegrees();
    this.cameraBearingDegrees =
        fromFront.minus(Rotation2d.fromRadians(cameraYawRadians)).getDegrees();
  }

  /** @return where the other robot is on the field, in meters */
  public Translation2d getFieldPosition() {
    return fieldPosition;
  }

  /** @return its bumper color, or UNKNOWN */
  public AllianceColor getAlliance() {
    return alliance;
  }

  /** @return when the camera saw it, in FPGA seconds */
  public double getTimestampSeconds() {
    return timestampSeconds;
  }

  /** @return straight-line distance from your robot's center, in meters */
  public double getDistanceMeters() {
    return distanceMeters;
  }

  /**
   * @return direction to it from your robot's front, in degrees from -180 to 180. 0 is straight
   *     ahead of the robot's front, positive is to the left.
   */
  public double getBearingDegrees() {
    return bearingDegrees;
  }

  /**
   * @return direction to it from the way the camera points, in degrees from -180 to 180. 0 is
   *     straight ahead of the camera, positive is to the left.
   */
  public double getCameraBearingDegrees() {
    return cameraBearingDegrees;
  }
}
