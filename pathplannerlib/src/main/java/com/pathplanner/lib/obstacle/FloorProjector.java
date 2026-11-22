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
    // The pixel's ray in the camera's frame: x out of the lens, y left, z up. Using both image
    // axes together keeps targets near the sides of the image at the right distance.
    double x = 1.0;
    double y = (cx - u) / fx;
    double z = (cy - v) / fy;

    // Tilt the camera down
    double xt = x * Math.cos(cameraPitchDownRad) + z * Math.sin(cameraPitchDownRad);
    double zt = -x * Math.sin(cameraPitchDownRad) + z * Math.cos(cameraPitchDownRad);
    if (zt >= -1e-9) {
      return null; // ray points at or above the horizon
    }
    double t = cameraHeightM / -zt;

    // Floor point relative to the lens, turned the way the camera faces
    double yawCos = Math.cos(cameraYawRad);
    double yawSin = Math.sin(cameraYawRad);
    double relX = cameraForwardM + t * (xt * yawCos - y * yawSin);
    double relY = cameraLeftM + t * (xt * yawSin + y * yawCos);

    // Then onto the field with the robot's pose
    double headingCos = Math.cos(robotHeadingRad);
    double headingSin = Math.sin(robotHeadingRad);
    return new Point(
        robotX + relX * headingCos - relY * headingSin,
        robotY + relX * headingSin + relY * headingCos);
  }
}
