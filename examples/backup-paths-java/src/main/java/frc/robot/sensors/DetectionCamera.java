package frc.robot.sensors;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import com.pathplanner.lib.obstacle.AllianceColor;
import com.pathplanner.lib.obstacle.FloorProjector;
import com.pathplanner.lib.obstacle.GamePiece;
import com.pathplanner.lib.obstacle.Obstacle;
import com.pathplanner.lib.obstacle.Point;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.sensors.DetectionCameraConfig.Role;

/**
 * The object detection camera (PhotonVision on the Orange Pi). Reports robots for backup paths
 * and game pieces for pickups.
 *
 * <p>Each detection is placed on the floor using the bottom-center of its box, the robot pose, the
 * camera mount from detection-camera.json, and the lens numbers from PhotonVision's calibration
 * (or from the file if fx, fy, cx, cy are given there).
 */
public class DetectionCamera implements DetectionSource {

    private final DetectionCameraConfig config;
    private final PhotonCamera camera;
    private final Supplier<Pose2d> poseSupplier;
    private FloorProjector projector;
    private double cx, cy;
    private boolean warnedNoCalibration = false;

    private List<Obstacle> robots = List.of();
    private List<GamePiece> pieces = List.of();

    /**
     * @param config camera settings from detection-camera.json
     * @param poseSupplier the robot pose from odometry
     */
    public DetectionCamera(DetectionCameraConfig config, Supplier<Pose2d> poseSupplier) {
        this.config = config;
        this.poseSupplier = poseSupplier;
        this.camera = new PhotonCamera(config.cameraName);
        if (config.hasIntrinsics) {
            makeProjector(config.fx, config.fy, config.cx, config.cy);
        }
    }

    @Override
    public List<Obstacle> getObstacles() {
        update();
        return robots;
    }

    @Override
    public List<GamePiece> getGamePieces() {
        update();
        return pieces;
    }

    /** Read new camera frames, if any, and refresh both lists from the newest one. */
    private void update() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.isEmpty() || !haveProjector()) {
            return;
        }
        PhotonPipelineResult latest = results.get(results.size() - 1);
        Pose2d pose = poseSupplier.get();
        double timestamp = latest.getTimestampSeconds();

        List<Obstacle> newRobots = new ArrayList<>();
        List<GamePiece> newPieces = new ArrayList<>();
        for (PhotonTrackedTarget target : latest.getTargets()) {
            double confidence = target.getDetectedObjectConfidence();
            // A negative confidence means PhotonVision didn't report one
            if (confidence >= 0 && confidence < config.minConfidence) continue;

            Role role = config.roleOf(target.getDetectedObjectClassID());
            if (role == Role.IGNORE) continue;

            Point bottom = bottomCenter(target);
            Point field = projector.projectToField(
                bottom.x(), bottom.y(), pose.getX(), pose.getY(), pose.getRotation().getRadians());
            if (field == null) continue; // Above the horizon, can't be on the floor

            switch (role) {
                case GAME_PIECE -> newPieces.add(new GamePiece(field, timestamp));
                case ROBOT_RED -> newRobots.add(new Obstacle(field, AllianceColor.RED, timestamp));
                case ROBOT_BLUE -> newRobots.add(new Obstacle(field, AllianceColor.BLUE, timestamp));
                case ROBOT -> newRobots.add(new Obstacle(field, AllianceColor.UNKNOWN, timestamp));
                default -> { }
            }
        }
        robots = newRobots;
        pieces = newPieces;
    }

    /** Use the file's lens numbers, or PhotonVision's calibration once it's available. */
    private boolean haveProjector() {
        if (projector != null) return true;
        Optional<Matrix<N3, N3>> matrix = camera.getCameraMatrix();
        if (matrix.isPresent()) {
            Matrix<N3, N3> m = matrix.get();
            makeProjector(m.get(0, 0), m.get(1, 1), m.get(0, 2), m.get(1, 2));
            return true;
        }
        if (!warnedNoCalibration) {
            warnedNoCalibration = true;
            DriverStation.reportWarning(
                "Detection camera " + config.cameraName + " has no calibration in PhotonVision. "
                    + "Calibrate it at the resolution the detection pipeline uses, or put fx, fy, cx, cy "
                    + "in detection-camera.json. Detections are ignored until then.", false);
        }
        return false;
    }

    private void makeProjector(double fx, double fy, double cx, double cy) {
        this.cx = cx;
        this.cy = cy;
        projector = new FloorProjector(
            config.heightMeters, config.pitchDownRadians, config.yawRadians,
            config.forwardMeters, config.leftMeters, fx, fy, cx, cy);
    }

    /** Middle of the bottom edge of the box, in image pixels. Falls back to the image center. */
    private Point bottomCenter(PhotonTrackedTarget target) {
        List<TargetCorner> corners = target.getDetectedCorners();
        if (corners.isEmpty()) {
            return new Point(cx, cy);
        }
        double maxY = Double.NEGATIVE_INFINITY;
        for (TargetCorner c : corners) {
            maxY = Math.max(maxY, c.y);
        }
        double sumX = 0;
        int count = 0;
        for (TargetCorner c : corners) {
            if (c.y >= maxY - 2) {
                sumX += c.x;
                count++;
            }
        }
        return new Point(sumX / count, maxY);
    }
}
