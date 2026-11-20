package com.pathplanner.lib.obstacle;

/**
 * Projects an image pixel onto the field floor, using the robot pose from odometry and the camera's
 * known mount. Assumes the target touches the floor, so the bottom-center of its bounding box is
 * the point to project.
 *
 * <p>Angle conventions: heading and yaw are counter-clockwise from the robot's forward axis.
 * Pitch is positive when the camera looks down. Image x grows to the right, image y grows down.
 */
public final class FloorProjector {
  private final double cameraHeightM;
  private final double cameraPitchDownRad;
  private final double cameraYawRad;
  private final double cameraForwardM;
  private final double cameraLeftM;
  private final double fx;
  private final double fy;
  private final double cx;
  private final double cy;

  /**
   * @param cameraHeightM height of the lens above the floor
   * @param cameraPitchDownRad pitch of the optical axis below horizontal, positive is down
   * @param cameraYawRad yaw of the optical axis relative to robot forward, positive is left
   * @param cameraForwardM lens position forward of the robot center
   * @param cameraLeftM lens position left of the robot center
   * @param fx focal length in pixels, x
   * @param fy focal length in pixels, y
   * @param cx principal point x in pixels
   * @param cy principal point y in pixels
   */
  public FloorProjector(
      double cameraHeightM,
      double cameraPitchDownRad,
      double cameraYawRad,
      double cameraForwardM,
      double cameraLeftM,
      double fx,
      double fy,
      double cx,
      double cy) {
    this.cameraHeightM = cameraHeightM;
    this.cameraPitchDownRad = cameraPitchDownRad;
    this.cameraYawRad = cameraYawRad;
    this.cameraForwardM = cameraForwardM;
    this.cameraLeftM = cameraLeftM;
    this.fx = fx;
    this.fy = fy;
    this.cx = cx;
    this.cy = cy;
  }

  /**
   * Project a pixel onto the floor.
   *
   * @param u image x in pixels
   * @param v image y in pixels (bottom of the target box)
   * @param robotX robot x on the field, meters
   * @param robotY robot y on the field, meters
   * @param robotHeadingRad robot heading, radians, counter-clockwise
   * @return field position of the floor point, or null if the ray does not hit the floor
   */
  public Point projectToField(
      double u, double v, double robotX, double robotY, double robotHeadingRad) {
    // Angle of this pixel's ray below the optical axis (positive = below)
    double rayDepressionOffset = Math.atan2(v - cy, fy);
    double depression = cameraPitchDownRad + rayDepressionOffset;
    if (depression <= 1e-6) {
      return null; // ray points at or above the horizon
    }
    double groundDistance = cameraHeightM / Math.tan(depression);

    // Horizontal bearing of the ray relative to the optical axis (left of center is positive)
    double bearingOffset = Math.atan2(cx - u, fx);

    // Lens position in field coordinates
    double headingCos = Math.cos(robotHeadingRad);
    double headingSin = Math.sin(robotHeadingRad);
    double lensX =
        robotX + cameraForwardM * headingCos - cameraLeftM * headingSin;
    double lensY =
        robotY + cameraForwardM * headingSin + cameraLeftM * headingCos;

    // Field direction of the ray
    double rayHeading = robotHeadingRad + cameraYawRad + bearingOffset;
    return new Point(
        lensX + groundDistance * Math.cos(rayHeading),
        lensY + groundDistance * Math.sin(rayHeading));
  }
}
