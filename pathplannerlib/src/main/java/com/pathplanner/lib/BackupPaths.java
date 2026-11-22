package com.pathplanner.lib;

import com.pathplanner.lib.detection.Camera;
import com.pathplanner.lib.obstacle.GamePieceSettings;
import com.pathplanner.lib.obstacle.ObstacleSettings;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * The one place to set up path regions in robot code. Call these in your RobotContainer, after
 * configuring AutoBuilder and before building any autos:
 *
 * <pre>{@code
 * BackupPaths.setup(drivetrain::getPose, drivetrain::stop);
 *
 * // "Robot in the way" regions
 * BackupPaths.watchForRobots(
 *     Camera.limelight("limelight-front").mountedAt(0.30, 0, 0.75, 15, 0).redClasses(1).blueClasses(2));
 *
 * // "Game piece" regions
 * BackupPaths.pickUpGamePieces(
 *     Camera.limelight("limelight-intake").mountedAt(-0.35, 0, 0.45, 25, 180),
 *     intake::intakeCommand, intake::hasPiece, intake);
 * }</pre>
 *
 * <p>Each line is optional. Without a robot camera, robot regions just follow the path, and
 * without a game piece camera, game piece regions just follow the path.
 */
public final class BackupPaths {
  /** Default height of the middle of a robot above the floor, in meters */
  public static final double ROBOT_TARGET_HEIGHT = 0.4;

  /** Default height of the middle of a game piece above the floor, in meters */
  public static final double PIECE_TARGET_HEIGHT = 0.08;

  private static Supplier<Pose2d> pose = null;
  private static Runnable stop = () -> {};
  private static Camera robotCamera = null;
  private static Camera pieceCamera = null;

  private BackupPaths() {}

  /**
   * Tell path regions how to find the robot and stop it.
   *
   * @param robotPose your robot's pose on the field, usually {@code drivetrain::getPose}
   * @param stopDrive stops the drivetrain, used while waiting for a robot to move
   */
  public static void setup(Supplier<Pose2d> robotPose, Runnable stopDrive) {
    pose = robotPose;
    stop = stopDrive == null ? () -> {} : stopDrive;
    apply();
  }

  /**
   * Use this camera for "Robot in the way" regions.
   *
   * @param camera the camera, from {@link Camera#limelight}, {@link Camera#photon} or another
   *     factory on {@link Camera}
   */
  public static void watchForRobots(Camera camera) {
    robotCamera = camera;
    if (camera != null) camera.setDefaultTargetHeight(ROBOT_TARGET_HEIGHT);
    warnIfShared();
    apply();
  }

  /**
   * Use this camera and intake for "Game piece" regions. The robot drives to a piece seen in the
   * region's orange zone, runs the intake, and rejoins the path.
   *
   * @param camera the camera that sees game pieces
   * @param intakeCommand makes the command that runs your intake, such as {@code
   *     intake::intakeCommand}
   * @param hasPiece true once the piece is in, such as a beam break. Can be null: then the intake
   *     runs for half a second after reaching the piece.
   * @param intakeSubsystems your intake subsystem(s), so the pickup can use them
   */
  public static void pickUpGamePieces(
      Camera camera,
      Supplier<Command> intakeCommand,
      BooleanSupplier hasPiece,
      Subsystem... intakeSubsystems) {
    pieceCamera = camera;
    if (camera != null) camera.setDefaultTargetHeight(PIECE_TARGET_HEIGHT);
    warnIfShared();
    GamePieceSettings.configure(
        camera,
        intakeCommand,
        hasPiece,
        0.5,
        3.0,
        GamePieceSettings.getIntakeFacingDegrees(),
        intakeSubsystems);
    // The camera drops old sightings itself, using its own maxAge
    GamePieceSettings.setStaleAfterSec(Double.POSITIVE_INFINITY);
  }

  /** Call this if the intake is on the back of the robot, so the robot backs onto pieces. */
  public static void intakeIsOnBack() {
    GamePieceSettings.setIntakeFacingDegrees(180);
  }

  /**
   * Optional switches, for example from dashboard choosers or a "safe mode" button.
   *
   * @param allowDivert when false, regions set to divert just follow the path instead
   * @param allowWait when false, regions never wait for a robot to move
   * @param allowPickups when false, game piece regions just follow the path
   */
  public static void setSwitches(
      BooleanSupplier allowDivert, BooleanSupplier allowWait, BooleanSupplier allowPickups) {
    ObstacleSettings.setSwitches(allowDivert, allowWait);
    GamePieceSettings.setEnabled(allowPickups);
  }

  /** Use this path instead of each region's own divert path, for example from a chooser. */
  public static void overrideDivertPath(Supplier<PathPlannerPath> divertPath) {
    ObstacleSettings.setDivertOverride(divertPath);
  }

  /** @return the robot pose given to {@link #setup}, or null before it is called */
  public static Supplier<Pose2d> getPoseSupplier() {
    return pose;
  }

  /** @return the camera watching for robots, or null */
  public static Camera getRobotCamera() {
    return robotCamera;
  }

  /** @return the camera watching for game pieces, or null */
  public static Camera getPieceCamera() {
    return pieceCamera;
  }

  /** Forget everything, for tests */
  static void reset() {
    pose = null;
    stop = () -> {};
    robotCamera = null;
    pieceCamera = null;
    ObstacleSettings.configure(null, null, () -> {}, 0.25);
    ObstacleSettings.setSwitches(null, null);
    ObstacleSettings.setDivertOverride(null);
    GamePieceSettings.configure(null, null, null, 0.5, 3.0, 0);
    GamePieceSettings.setEnabled(null);
    GamePieceSettings.setStaleAfterSec(null);
  }

  private static void apply() {
    // The camera drops old sightings itself, using its own maxAge, so nothing is dropped twice
    ObstacleSettings.configure(robotCamera, pose, stop, Double.POSITIVE_INFINITY);
  }

  private static void warnIfShared() {
    if (robotCamera != null && robotCamera == pieceCamera) {
      DriverStation.reportWarning(
          "BackupPaths: the same Camera is used for robots and game pieces. Make two Camera"
              + " objects (they can read the same physical camera) with their own onlyClasses(...),"
              + " so pieces are not treated as robots and robots are not picked up.",
          false);
    }
  }
}
