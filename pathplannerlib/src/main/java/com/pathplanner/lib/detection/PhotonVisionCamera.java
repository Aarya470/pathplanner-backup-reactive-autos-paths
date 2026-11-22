package com.pathplanner.lib.detection;

import com.pathplanner.lib.obstacle.FloorProjector;
import com.pathplanner.lib.obstacle.Point;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/**
 * A PhotonVision object detection camera. Made with {@link Camera#photon(String)}. Kept in its own
 * class so teams without photonlib can still use the other cameras.
 *
 * <p>If the camera is calibrated, the bottom of each box is placed on the floor using the lens
 * calibration, which is the most accurate. Otherwise the angles to the middle of the box are used,
 * like a Limelight.
 */
final class PhotonVisionCamera extends Camera {
  private final PhotonCamera camera;
  private FloorProjector projector = null;

  PhotonVisionCamera(String name) {
    this.camera = new PhotonCamera(name);
  }

  @Override
  protected Reading read() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      if (!camera.isConnected()) {
        return Reading.problem(
            "No data from " + camera.getName() + " (check the name, and that PhotonLib matches the"
                + " PhotonVision version)");
      }
      return null; // no new frame
    }
    PhotonPipelineResult latest = results.get(results.size() - 1);
    double timestamp = latest.getTimestampSeconds();

    FloorProjector floor = projector();
    Pose2d robot = Camera.robotPose().get();
    String noteIfEmpty = null;
    List<CameraTarget> angles = new ArrayList<>();
    List<Placed> placed = new ArrayList<>();

    for (PhotonTrackedTarget target : latest.getTargets()) {
      int classId = target.getDetectedObjectClassID();
      double confidence = target.getDetectedObjectConfidence();
      if (confidence >= 0 && confidence < getMinConfidence()) {
        noteIfEmpty = "Low confidence " + String.format("%.2f", confidence);
        continue;
      }
      List<TargetCorner> corners = target.getMinAreaRectCorners();
      if (corners == null || corners.isEmpty()) corners = target.getDetectedCorners();

      if (floor != null && corners != null && !corners.isEmpty()) {
        double minX = corners.get(0).x, maxX = corners.get(0).x, maxY = corners.get(0).y;
        for (TargetCorner c : corners) {
          minX = Math.min(minX, c.x);
          maxX = Math.max(maxX, c.x);
          maxY = Math.max(maxY, c.y);
        }
        Point p =
            floor.projectToField(
                (minX + maxX) / 2.0,
                maxY,
                robot.getX(),
                robot.getY(),
                robot.getRotation().getRadians());
        if (p != null) {
          placed.add(new Placed(new Translation2d(p.x(), p.y()), classId));
        } else {
          noteIfEmpty = "Box bottom above the horizon (check height and tilt)";
        }
        continue;
      }
      // PhotonVision yaw is positive to the right and pitch positive up. Its pitch is measured
      // after the yaw turn, so convert it to the image-plane angle that Limelight's ty uses.
      double yaw = target.getYaw();
      angles.add(new CameraTarget(classId, yaw, pitchToTy(yaw, target.getPitch())));
    }
    return new Reading(angles, placed, timestamp, null, noteIfEmpty);
  }

  /** Turn PhotonVision's pitch, measured after the yaw turn, into an image-plane ty */
  static double pitchToTy(double yawDegrees, double pitchDegrees) {
    return Math.toDegrees(
        Math.atan(Math.tan(Math.toRadians(pitchDegrees)) / Math.cos(Math.toRadians(yawDegrees))));
  }

  @Override
  protected void onMountChanged() {
    projector = null;
  }

  /** The calibrated floor projection. Null until the camera is mounted and calibrated. */
  private FloorProjector projector() {
    if (!mounted) return null;
    if (projector != null) return projector;
    Optional<Matrix<N3, N3>> matrix = camera.getCameraMatrix();
    if (matrix.isEmpty()) return null;
    Matrix<N3, N3> m = matrix.get();
    projector =
        new FloorProjector(
            heightMeters,
            Math.toRadians(tiltDownDegrees),
            Math.toRadians(facingDegrees),
            forwardMeters,
            leftMeters,
            m.get(0, 0),
            m.get(1, 1),
            m.get(0, 2),
            m.get(1, 2));
    return projector;
  }
}
