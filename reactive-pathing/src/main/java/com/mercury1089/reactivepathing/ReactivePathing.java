package com.mercury1089.reactivepathing;

import com.mercury1089.detection.Camera;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.List;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Reactive Autonomous Pathing: makes PathPlanner autos react to what the cameras see, using the
 * regions you draw in the PathPlanner Backup Paths app.
 *
 * <p>Setup, in RobotContainer:
 *
 * <ol>
 *   <li>Replace {@code AutoBuilder.configure(...)} with {@code ReactivePathing.configure(...)}, with
 *       the same arguments.
 *   <li>Give it cameras: {@code ReactivePathing.watchForRobots(camera)} and {@code
 *       ReactivePathing.pickUpGamePieces(camera, intake::intakeCommand, intake::hasPiece)}.
 *   <li>Build autos the normal PathPlanner way, for example with {@code
 *       AutoBuilder.buildAutoChooser()}.
 * </ol>
 */
public class ReactivePathing {
  // How the robot drives, from configure(...)
  static Supplier<Pose2d> robotPose = null;
  static Supplier<ChassisSpeeds> robotSpeeds = null;
  static BiConsumer<ChassisSpeeds, DriveFeedforwards> drive = null;
  static PathFollowingController controller = null;
  static RobotConfig robotConfig = null;
  static BooleanSupplier shouldFlip = () -> false;
  static Subsystem[] driveSubsystems = new Subsystem[0];

  // Cameras and intake
  static Camera robotCamera = null;
  static Camera gamePieceCamera = null;
  static Supplier<Command> intakeCommand = null;
  static BooleanSupplier hasGamePiece = null;
  static boolean intakeOnBack = false;

  private ReactivePathing() {}

  // ---------------------------------------------------------------- setup

  /**
   * Set up path following. Use this instead of {@code AutoBuilder.configure(...)}, with the same
   * arguments. Paths with regions then react to the cameras, and all other paths run like normal.
   */
  public static void configure(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
      BiConsumer<ChassisSpeeds, DriveFeedforwards> output,
      PathFollowingController controller,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath,
      Subsystem... driveRequirements) {
    ReactivePathing.robotPose = poseSupplier;
    ReactivePathing.robotSpeeds = robotRelativeSpeedsSupplier;
    ReactivePathing.drive = output;
    ReactivePathing.controller = controller;
    ReactivePathing.robotConfig = robotConfig;
    ReactivePathing.shouldFlip = shouldFlipPath;
    ReactivePathing.driveSubsystems = driveRequirements;
    giveCamerasThePose();

    // Load the field for pathfinding now, instead of in the middle of an auto
    Pathfinding.ensureInitialized();

    // PathPlanner asks commandForPath for the command of every path in an auto
    AutoBuilder.configureCustom(
        ReactivePathing::commandForPath,
        poseSupplier,
        resetPose,
        shouldFlipPath,
        robotConfig.isHolonomic);
  }

  /** The same as the other configure(...), for drivetrains that don't use feedforwards. */
  public static void configure(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
      Consumer<ChassisSpeeds> output,
      PathFollowingController controller,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath,
      Subsystem... driveRequirements) {
    configure(
        poseSupplier,
        resetPose,
        robotRelativeSpeedsSupplier,
        (speeds, feedforwards) -> output.accept(speeds),
        controller,
        robotConfig,
        shouldFlipPath,
        driveRequirements);
  }

  /** Use this camera for "robot in the way" regions. */
  public static void watchForRobots(Camera camera) {
    robotCamera = camera;
    warnIfNoClasses(camera);
    giveCamerasThePose();
  }

  /**
   * Use this camera and intake for "game piece" regions. In a region, the robot drives to a game
   * piece seen inside the orange zone with the intake running, then goes back to the path.
   *
   * @param camera the camera that sees game pieces (it can be the same camera as for robots)
   * @param intakeCommand makes the command that runs the intake, such as {@code
   *     intake::intakeCommand}
   * @param hasGamePiece true once the piece is in, such as a beam break. Can be null: then the
   *     intake runs for half a second after reaching the piece.
   */
  public static void pickUpGamePieces(
      Camera camera, Supplier<Command> intakeCommand, BooleanSupplier hasGamePiece) {
    gamePieceCamera = camera;
    ReactivePathing.intakeCommand = intakeCommand;
    ReactivePathing.hasGamePiece = hasGamePiece;
    warnIfNoClasses(camera);
    giveCamerasThePose();
  }

  /** Call this if the intake is on the back of the robot, so the robot backs onto pieces. */
  public static void intakeIsOnBack() {
    intakeOnBack = true;
  }

  // ---------------------------------------------------------------- pathfinding

  /**
   * Drive to a pose, going around the field's obstacles. Use this instead of {@code
   * AutoBuilder.pathfindToPose(...)}, which doesn't work after {@link #configure}.
   */
  public static Command pathfindToPose(Pose2d pose, PathConstraints constraints) {
    return pathfindToPose(pose, constraints, 0.0);
  }

  /** The same, arriving at this speed, in meters per second. */
  public static Command pathfindToPose(
      Pose2d pose, PathConstraints constraints, double endSpeed) {
    checkConfigured();
    return new PathfindingCommand(
        pose,
        constraints,
        endSpeed,
        robotPose,
        robotSpeeds,
        drive,
        controller,
        robotConfig,
        driveSubsystems);
  }

  /** Drive to a pose drawn on the blue side. On the red alliance, it is flipped to red. */
  public static Command pathfindToPoseFlipped(Pose2d bluePose, PathConstraints constraints) {
    return pathfindToPoseFlipped(bluePose, constraints, 0.0);
  }

  /** The same, arriving at this speed, in meters per second. */
  public static Command pathfindToPoseFlipped(
      Pose2d bluePose, PathConstraints constraints, double endSpeed) {
    // Commands.defer waits until the command starts to check the alliance
    return Commands.defer(
        () -> {
          Pose2d pose = bluePose;
          if (shouldFlip.getAsBoolean()) {
            pose = FlippingUtil.flipFieldPose(bluePose);
          }
          return pathfindToPose(pose, constraints, endSpeed);
        },
        Set.of(driveSubsystems));
  }

  /** Drive to the start of a path, going around the field's obstacles, then follow it. */
  public static Command pathfindThenFollowPath(PathPlannerPath path, PathConstraints constraints) {
    checkConfigured();
    return new PathfindThenFollowPath(
        path,
        constraints,
        robotPose,
        robotSpeeds,
        drive,
        controller,
        robotConfig,
        shouldFlip,
        driveSubsystems);
  }

  // ---------------------------------------------------------------- inside

  /** The command for one path of an auto. Paths with no regions run like normal. */
  static Command commandForPath(PathPlannerPath path) {
    // Paths made in code and Choreo paths aren't drawn in the app, so they have no regions
    boolean hasFile = path.name != null && !path.name.isEmpty() && !path.isChoreoPath();
    if (!hasFile) {
      return followPath(path);
    }
    List<Region> regions = Region.forPath(path);
    if (regions.isEmpty()) {
      return followPath(path);
    }
    // Work out the divert paths now, while the auto is built, instead of in the middle of it
    for (Region region : regions) {
      if (region.divertPath != null) {
        region.divertPath.getIdealTrajectory(robotConfig);
      }
    }
    return new ReactivePathCommand(path, regions);
  }

  /** Follow a path like PathPlanner normally does. */
  static Command followPath(PathPlannerPath path) {
    checkConfigured();
    return new FollowPathCommand(
        path, robotPose, robotSpeeds, drive, controller, robotConfig, shouldFlip, driveSubsystems);
  }

  /** Stop the drivetrain. */
  static void stopDriving() {
    drive.accept(new ChassisSpeeds(), DriveFeedforwards.zeros(robotConfig.numModules));
  }

  private static void checkConfigured() {
    if (robotConfig == null) {
      throw new IllegalStateException(
          "Call ReactivePathing.configure(...) in RobotContainer before building any autos.");
    }
  }

  private static void warnIfNoClasses(Camera camera) {
    if (camera != null && !camera.hasClassesSet()) {
      DriverStation.reportWarning(
          "Reactive Pathing: a camera has no classes set, so it will never see robots or game"
              + " pieces. Add robotClasses(...) or gamePieceClasses(...) to it.",
          false);
    }
  }

  /** The cameras need the robot's pose to give field positions. */
  private static void giveCamerasThePose() {
    if (robotPose == null) {
      return;
    }
    if (robotCamera != null) {
      robotCamera.usePose(robotPose);
    }
    if (gamePieceCamera != null) {
      gamePieceCamera.usePose(robotPose);
    }
  }

  /** Forget everything, for tests. */
  static void reset() {
    robotPose = null;
    robotSpeeds = null;
    drive = null;
    controller = null;
    robotConfig = null;
    shouldFlip = () -> false;
    driveSubsystems = new Subsystem[0];
    robotCamera = null;
    gamePieceCamera = null;
    intakeCommand = null;
    hasGamePiece = null;
    intakeOnBack = false;
    AutoBuilder.resetForTesting();
  }
}
