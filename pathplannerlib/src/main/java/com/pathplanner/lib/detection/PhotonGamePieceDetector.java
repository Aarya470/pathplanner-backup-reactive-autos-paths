package com.pathplanner.lib.detection;

import com.pathplanner.lib.obstacle.FloorProjector;
import com.pathplanner.lib.obstacle.GamePiece;
import com.pathplanner.lib.obstacle.GamePieceSource;
import com.pathplanner.lib.obstacle.Point;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
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
 * A {@link GamePieceSource} for a PhotonVision object detection camera, for game piece path
 * regions.
 *
 * <p>Each box the model draws around a game piece is turned into a field position: the bottom
 * middle of the box is where the piece touches the floor, and the camera's height, tilt and lens
 * calibration give the distance to it. Your robot's pose places it on the field. A path region
 * picks a piece up only if it is inside the region's pickup zone.
 *
 * <p>Needs the photonlib vendordep, and the camera calibrated in PhotonVision at the resolution
 * the detection pipeline uses. Until it is calibrated, nothing is reported.
 *
 * <pre>{@code
 * PhotonGamePieceDetector pieces = new PhotonGamePieceDetector(
 *     "IntakeCam", robotToCamera, () -> drivetrain.getPose(), 0);
 * GamePieceSettings.configure(
 *     pieces, intake::runCommand, intake::hasPiece, 0.3, 3.0, 180, intake);
 * }</pre>
 */
public class PhotonGamePieceDetector implements GamePieceSource {
  private final PhotonCamera camera;
  private final Transform3d robotToCamera;
  private final Supplier<Pose2d> robotPose;
  private final int[] pieceClassIds;

  private double minConfidence = 0.5;
  private double maxAgeSeconds = 0.25;

  private FloorProjector projector = null;
  private boolean warnedNoCalibration = false;
  private double lastPollTime = -1;
  private List<GamePiece> pieces = new ArrayList<>();
  private String status = "No frames yet";

  /**
   * @param cameraName the camera's name in PhotonVision, exactly as typed there
   * @param robotToCamera where the camera sits on the robot: x forward, y left, z up from the
   *     robot's center on the floor, and the rotation's yaw for which way it points (0 = forward,
   *     pi = backward) and pitch for its tilt (positive = tilted down)
   * @param robotPose your robot's pose on the field, usually from the pose estimator
   * @param pieceClassIds the class number or numbers your model uses for game pieces
   */
  public PhotonGamePieceDetector(
      String cameraName,
      Transform3d robotToCamera,
      Supplier<Pose2d> robotPose,
      int... pieceClassIds) {
    this.camera = new PhotonCamera(cameraName);
    this.robotToCamera = robotToCamera;
    this.robotPose = robotPose;
    this.pieceClassIds = pieceClassIds;
  }

  /** Boxes below this confidence (0 to 1) are ignored. Default 0.5. */
  public void setMinConfidence(double minConfidence) {
    this.minConfidence = minConfidence;
  }

  /** Pieces seen longer ago than this many seconds are dropped. Default 0.25. */
  public void setMaxAgeSeconds(double maxAgeSeconds) {
    this.maxAgeSeconds = maxAgeSeconds;
  }

  @Override
  public List<GamePiece> getGamePieces() {
    poll();
    double now = Timer.getFPGATimestamp();
    List<GamePiece> fresh = new ArrayList<>();
    for (GamePiece piece : pieces) {
      if (now - piece.timestampSec() <= maxAgeSeconds) {
        fresh.add(piece);
      }
    }
    return fresh;
  }

  /** @return true if PhotonVision is sending data for this camera */
  public boolean isConnected() {
    return camera.isConnected();
  }

  /** @return true once PhotonVision has a calibration for the resolution in use */
  public boolean isCalibrated() {
    return projector != null || camera.getCameraMatrix().isPresent();
  }

  /**
   * @return a short note on the last frame: "OK", or why boxes were dropped, such as "Wrong class
   *     ID 1" or "Not calibrated". Useful on a dashboard while setting up.
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
    List<GamePiece> found = new ArrayList<>();
    String note = latest.getTargets().isEmpty() ? "No boxes" : "OK";

    for (PhotonTrackedTarget target : latest.getTargets()) {
      int classId = target.getDetectedObjectClassID();
      if (!contains(pieceClassIds, classId)) {
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
      found.add(new GamePiece(field, timestamp));
    }

    if (!found.isEmpty()) {
      note = "OK";
    }
    status = note;
    pieces = found;
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
            "PhotonGamePieceDetector: camera "
                + camera.getName()
                + " has no calibration in PhotonVision for the resolution in use. Game pieces are"
                + " not reported until it does.",
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

  private static boolean contains(int[] ids, int id) {
    for (int value : ids) {
      if (value == id) {
        return true;
      }
    }
    return false;
  }
}
