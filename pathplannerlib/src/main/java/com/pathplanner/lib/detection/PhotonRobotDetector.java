package com.pathplanner.lib.detection;

import com.pathplanner.lib.obstacle.AllianceColor;
import com.pathplanner.lib.obstacle.FloorProjector;
import com.pathplanner.lib.obstacle.Point;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/**
 * A {@link RobotDetector} for a PhotonVision object detection camera.
 *
 * <p>Each box the model draws around a robot is turned into a field position: the bottom middle
 * of the box is where that robot touches the floor, and the camera's height, tilt and lens
 * calibration give the distance to it. Your robot's pose places it on the field.
 *
 * <p>Needs the photonlib vendordep, and the camera calibrated in PhotonVision at the resolution
 * the detection pipeline uses. Until it is calibrated, nothing is reported.
 *
 * <pre>{@code
 * PhotonRobotDetector detector = new PhotonRobotDetector(
 *     "DetectionCam", robotToCamera, () -> drivetrain.getPose(), 1);
 * if (detector.isRobotAhead(1.5)) { ... }
 * }</pre>
 *
 * @deprecated Simpler: {@code Camera.photon("name").mountedAt(...)} with {@link
 *     com.pathplanner.lib.BackupPaths}. This class still works.
 */
@Deprecated
public class PhotonRobotDetector implements RobotDetector {
  private final PhotonCamera camera;
  private final Transform3d robotToCamera;
  private final Supplier<Pose2d> robotPose;
  private final int[] robotClassIds;

  private int[] redClassIds = new int[0];
  private int[] blueClassIds = new int[0];
  private double minConfidence = 0.5;
  private double maxAgeSeconds = 0.25;
  private double aheadHalfAngleDegrees = 30.0;

  private FloorProjector projector = null;
  private boolean warnedNoCalibration = false;
  private double lastPollTime = -1;
  private List<Sighting> sightings = new ArrayList<>();

  private int targetsInFrame = 0;
  private String status = "No frames yet";

  private static class Sighting {
    final Translation2d position;
    final AllianceColor alliance;
    final double timestamp;

    Sighting(Translation2d position, AllianceColor alliance, double timestamp) {
      this.position = position;
      this.alliance = alliance;
      this.timestamp = timestamp;
    }
  }

  /**
   * @param cameraName the camera's name in PhotonVision, exactly as typed there
   * @param robotToCamera where the camera sits on the robot, the same way PhotonVision's pose
   *     estimator takes it: x forward, y left, z up from the robot's center on the floor, and the
   *     rotation's yaw for which way it points (0 = forward) and pitch for its tilt (positive =
   *     tilted down)
   * @param robotPose your robot's pose on the field, usually from the pose estimator
   * @param robotClassIds the class number or numbers your model uses for robots
   */
  public PhotonRobotDetector(
      String cameraName,
      Transform3d robotToCamera,
      Supplier<Pose2d> robotPose,
      int... robotClassIds) {
    this.camera = new PhotonCamera(cameraName);
    this.robotToCamera = robotToCamera;
    this.robotPose = robotPose;
    this.robotClassIds = robotClassIds;
  }

  /**
   * Which of the robot classes mean a red or blue robot, so path regions can tell teammates from
   * opponents. Without this, every robot's color is UNKNOWN and only regions set to "any" react.
   */
  public void setAllianceClassIds(int[] redClassIds, int[] blueClassIds) {
    this.redClassIds = redClassIds;
    this.blueClassIds = blueClassIds;
  }

  /** Boxes below this confidence (0 to 1) are ignored. Default 0.5. */
  public void setMinConfidence(double minConfidence) {
    this.minConfidence = minConfidence;
  }

  /** Detections older than this many seconds are ignored. Default 0.25. */
  public void setMaxAgeSeconds(double maxAgeSeconds) {
    this.maxAgeSeconds = maxAgeSeconds;
  }

  /** How far to each side of the camera's direction counts as "ahead". Default 30 degrees. */
  public void setAheadHalfAngleDegrees(double aheadHalfAngleDegrees) {
    this.aheadHalfAngleDegrees = aheadHalfAngleDegrees;
  }

  @Override
  public double getAheadHalfAngleDegrees() {
    return aheadHalfAngleDegrees;
  }

  @Override
  public double getMaxAgeSeconds() {
    return maxAgeSeconds;
  }

  @Override
  public List<DetectedRobot> getRobots() {
    poll();
    double now = Timer.getFPGATimestamp();
    Pose2d pose = robotPose.get();
    double cameraYaw = robotToCamera.getRotation().getZ();
    List<DetectedRobot> robots = new ArrayList<>();
    for (Sighting sighting : sightings) {
      if (now - sighting.timestamp > maxAgeSeconds) {
        continue;
      }
      robots.add(
          new DetectedRobot(
              sighting.position, sighting.alliance, sighting.timestamp, pose, cameraYaw));
    }
    return robots;
  }

  /** @return true if PhotonVision is sending data for this camera */
  public boolean isConnected() {
    return camera.isConnected();
  }

  /** @return true once PhotonVision has a calibration for the resolution in use */
  public boolean isCalibrated() {
    return projector != null || camera.getCameraMatrix().isPresent();
  }

  /** @return how many boxes were in the last frame, before any filtering */
  public int getTargetsInFrame() {
    poll();
    return targetsInFrame;
  }

  /**
   * @return a short note on the last frame: "OK", or why boxes were dropped, such as "Wrong class
   *     ID 2" or "Not calibrated". Useful on a dashboard while setting up.
   */
  public String getStatus() {
    poll();
    return status;
  }

  private void poll() {
    double now = Timer.getFPGATimestamp();
    if (now - lastPollTime < 0.005) {
      return;
    }
    lastPollTime = now;

    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      return;
    }
    if (!makeProjector()) {
      status = "Not calibrated";
      return;
    }

    PhotonPipelineResult latest = results.get(results.size() - 1);
    Pose2d pose = robotPose.get();
    double timestamp = latest.getTimestampSeconds();
    List<Sighting> found = new ArrayList<>();
    targetsInFrame = latest.getTargets().size();
    String note = targetsInFrame == 0 ? "No boxes" : "OK";

    for (PhotonTrackedTarget target : latest.getTargets()) {
      int classId = target.getDetectedObjectClassID();
      if (!contains(robotClassIds, classId)) {
        note = "Wrong class ID " + classId;
        continue;
      }
      double confidence = target.getDetectedObjectConfidence();
      if (confidence >= 0 && confidence < minConfidence) {
        note = "Low confidence " + String.format("%.2f", confidence);
        continue;
      }
      List<TargetCorner> corners = target.getMinAreaRectCorners();
      if (corners == null || corners.isEmpty()) {
        corners = target.getDetectedCorners();
      }
      if (corners == null || corners.isEmpty()) {
        note = "No box corners";
        continue;
      }

      double minX = corners.get(0).x;
      double maxX = corners.get(0).x;
      double maxY = corners.get(0).y;
      for (TargetCorner corner : corners) {
        minX = Math.min(minX, corner.x);
        maxX = Math.max(maxX, corner.x);
        maxY = Math.max(maxY, corner.y);
      }
      Point field =
          projector.projectToField(
              (minX + maxX) / 2.0,
              maxY,
              pose.getX(),
              pose.getY(),
              pose.getRotation().getRadians());
      if (field == null) {
        note = "Box bottom above the horizon (check camera tilt)";
        continue;
      }
      found.add(
          new Sighting(new Translation2d(field.x(), field.y()), allianceOf(classId), timestamp));
    }

    if (!found.isEmpty()) {
      note = "OK";
    }
    status = note;
    sightings = found;
  }

  private boolean makeProjector() {
    if (projector != null) {
      return true;
    }
    Optional<Matrix<N3, N3>> matrix = camera.getCameraMatrix();
    if (matrix.isEmpty()) {
      if (!warnedNoCalibration) {
        warnedNoCalibration = true;
        DriverStation.reportWarning(
            "PhotonRobotDetector: camera "
                + camera.getName()
                + " has no calibration in PhotonVision for the resolution in use. Robots are not"
                + " reported until it does.",
            false);
      }
      return false;
    }
    Matrix<N3, N3> m = matrix.get();
    projector =
        new FloorProjector(
            robotToCamera.getZ(),
            robotToCamera.getRotation().getY(),
            robotToCamera.getRotation().getZ(),
            robotToCamera.getX(),
            robotToCamera.getY(),
            m.get(0, 0),
            m.get(1, 1),
            m.get(0, 2),
            m.get(1, 2));
    return true;
  }

  private AllianceColor allianceOf(int classId) {
    if (contains(redClassIds, classId)) {
      return AllianceColor.RED;
    }
    if (contains(blueClassIds, classId)) {
      return AllianceColor.BLUE;
    }
    return AllianceColor.UNKNOWN;
  }

  private static boolean contains(int[] ids, int id) {
    for (int value : ids) {
      if (value == id) {
        return true;
      }
    }
    return false;
  }
}
