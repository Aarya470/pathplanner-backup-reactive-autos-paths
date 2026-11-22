package com.mercury1089.reactivepathing;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.mercury1089.vision.Camera;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.HashSet;
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
 *   <li>Give it cameras from Mercury Vision: {@code ReactivePathing.watchForRobots(camera)} and,
 *       for game pieces, {@code ReactivePathing.pickUpGamePieces(camera, ...)}.
 *   <li>Build autos the normal PathPlanner way, for example with {@code
 *       AutoBuilder.buildAutoChooser()}.
 * </ol>
 *
 * <p>The first pathfinding command of a robot program is slow to start, because Java is still
 * warming up. To avoid that in your auto, schedule {@code PathfindingCommand.warmupCommand()} in
 * {@code Robot.robotInit()}.
 */
public class ReactivePathing {
  // How the robot drives, from configure(...)
  private static Supplier<Pose2d> robotPose = null;
  private static Supplier<ChassisSpeeds> robotSpeeds = null;
  private static BiConsumer<ChassisSpeeds, DriveFeedforwards> driveOutput = null;
  private static PathFollowingController controller = null;
  private static RobotConfig robotConfig = null;
  private static BooleanSupplier shouldFlip = () -> false;
  private static Subsystem[] driveSubsystems = new Subsystem[0];

  // Cameras and intake
  private static Camera robotCamera = null;
  private static Camera gamePieceCamera = null;
  private static Supplier<Command> intakeCommand = () -> Commands.none();
  private static BooleanSupplier hasGamePiece = null;
  private static Subsystem[] intakeSubsystems = new Subsystem[0];
  private static double intakeFacingDegrees = 0.0;
  private static double intakeAfterArrivingSeconds = 0.5;
  private static double maxPickupSeconds = 3.0;

  // Switches
  private static BooleanSupplier allowDivert = () -> true;
  private static BooleanSupplier allowWait = () -> true;
  private static BooleanSupplier allowPickups = () -> true;
  private static Supplier<PathPlannerPath> divertOverride = () -> null;

  private ReactivePathing() {}

  // ------------------------------------------------------------------ setup

  /**
   * Set up path following. Use this instead of {@code AutoBuilder.configure(...)}, with the same
   * arguments. Paths with regions then react to the cameras, and all other paths run exactly like
   * normal PathPlanner paths.
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
    ReactivePathing.driveOutput = output;
    ReactivePathing.controller = controller;
    ReactivePathing.robotConfig = robotConfig;
    ReactivePathing.shouldFlip = shouldFlipPath;
    ReactivePathing.driveSubsystems = driveRequirements;
    giveCamerasThePose();

    // Load the field's obstacles for pathfinding now, instead of in the middle of an auto
    Pathfinding.ensureInitialized();

    // PathPlanner asks this for the command of every path in an auto
    AutoBuilder.configureCustom(
        (path) -> commandForPath(path),
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
    warnIfNoClasses(camera, "watchForRobots");
    giveCamerasThePose();
  }

  /**
   * Use this camera and intake for "game piece" regions. In a region, the robot drives to a game
   * piece seen inside the orange zone, runs the intake, then goes back to the path.
   *
   * @param camera the camera that sees game pieces (it can be the same camera as for robots)
   * @param intakeCommand makes the command that runs the intake, for example {@code
   *     intake::intakeCommand}. It runs while driving to a piece and is stopped after. If another
   *     command (not your auto) is using a subsystem it needs, it isn't started, so that command
   *     (maybe your whole auto) isn't cancelled.
   * @param hasGamePiece true once the piece is in, such as a beam break. Can be null: then the
   *     intake runs for half a second after reaching the piece.
   * @param intakeRequirements your intake subsystem(s). Optional: the subsystems your intake
   *     command needs are what is checked.
   */
  public static void pickUpGamePieces(
      Camera camera,
      Supplier<Command> intakeCommand,
      BooleanSupplier hasGamePiece,
      Subsystem... intakeRequirements) {
    gamePieceCamera = camera;
    warnIfNoClasses(camera, "pickUpGamePieces");
    ReactivePathing.intakeCommand = intakeCommand;
    ReactivePathing.hasGamePiece = hasGamePiece;
    ReactivePathing.intakeSubsystems = intakeRequirements;
    giveCamerasThePose();
  }

  /** Call this if the intake is on the back of the robot, so the robot backs onto pieces. */
  public static void intakeIsOnBack() {
    intakeFacingDegrees = 180.0;
  }

  /**
   * Turn parts on and off, for example from dashboard choosers. Divert and wait are read when each
   * path starts. Pickups are checked each time a piece is seen.
   *
   * @param allowDivert when false, regions never divert
   * @param allowWait when false, regions never stop to wait
   * @param allowPickups when false, game piece regions are skipped
   */
  public static void setSwitches(
      BooleanSupplier allowDivert, BooleanSupplier allowWait, BooleanSupplier allowPickups) {
    ReactivePathing.allowDivert = allowDivert;
    ReactivePathing.allowWait = allowWait;
    ReactivePathing.allowPickups = allowPickups;
  }

  /**
   * Divert onto this path instead of each region's own divert path. Return null to not. It is
   * asked for when each auto is built, and again when the robot diverts.
   */
  public static void overrideDivertPath(Supplier<PathPlannerPath> divertPath) {
    divertOverride = divertPath;
  }

  // ------------------------------------------------------------------ pathfinding

  /**
   * Drive to a pose, avoiding the field's obstacles. Use this instead of {@code
   * AutoBuilder.pathfindToPose(...)}, which doesn't work after {@link #configure}.
   */
  public static Command pathfindToPose(Pose2d pose, PathConstraints constraints) {
    return pathfindToPose(pose, constraints, 0.0);
  }

  /** The same, ending at a speed in meters per second. */
  public static Command pathfindToPose(
      Pose2d pose, PathConstraints constraints, double goalEndSpeed) {
    checkConfigured();
    return new PathfindingCommand(
        pose,
        constraints,
        goalEndSpeed,
        robotPose,
        robotSpeeds,
        driveOutput,
        controller,
        robotConfig,
        driveSubsystems);
  }

  /** The same, with the end speed as a WPILib unit, such as {@code MetersPerSecond.of(1)}. */
  public static Command pathfindToPose(
      Pose2d pose, PathConstraints constraints, LinearVelocity goalEndSpeed) {
    return pathfindToPose(pose, constraints, goalEndSpeed.in(MetersPerSecond));
  }

  /** Drive to a pose drawn on the blue side, flipped to red when on the red alliance. */
  public static Command pathfindToPoseFlipped(Pose2d bluePose, PathConstraints constraints) {
    return pathfindToPoseFlipped(bluePose, constraints, 0.0);
  }

  /** The same, with the end speed as a WPILib unit, such as {@code MetersPerSecond.of(1)}. */
  public static Command pathfindToPoseFlipped(
      Pose2d bluePose, PathConstraints constraints, LinearVelocity goalEndSpeed) {
    return pathfindToPoseFlipped(bluePose, constraints, goalEndSpeed.in(MetersPerSecond));
  }

  /** The same, ending at a speed in meters per second. */
  public static Command pathfindToPoseFlipped(
      Pose2d bluePose, PathConstraints constraints, double goalEndSpeed) {
    return Commands.defer(
        () -> {
          Pose2d pose = bluePose;
          if (shouldFlip.getAsBoolean()) {
            pose = FlippingUtil.flipFieldPose(bluePose);
          }
          return pathfindToPose(pose, constraints, goalEndSpeed);
        },
        driveSet());
  }

  /** Drive to the start of a path, avoiding the field's obstacles, then follow it. */
  public static Command pathfindThenFollowPath(PathPlannerPath path, PathConstraints constraints) {
    checkConfigured();
    return new PathfindThenFollowPath(
        path,
        constraints,
        robotPose,
        robotSpeeds,
        driveOutput,
        controller,
        robotConfig,
        shouldFlip,
        driveSubsystems);
  }

  /** Follow a path like PathPlanner normally does, with no reacting. */
  public static Command followPathNormally(PathPlannerPath path) {
    checkConfigured();
    return new FollowPathCommand(
        path,
        robotPose,
        robotSpeeds,
        driveOutput,
        controller,
        robotConfig,
        shouldFlip,
        driveSubsystems);
  }

  // ------------------------------------------------------------------ used by the path command

  /** The command for one path of an auto. Paths with no regions run normally. */
  static Command commandForPath(PathPlannerPath path) {
    // Paths made in code (on-the-fly paths) have no file, so no regions
    if (path.name == null || path.name.isEmpty()) {
      return followPathNormally(path);
    }
    // Choreo paths aren't drawn in the app, so they have no regions
    if (path.isChoreoPath()) {
      return followPathNormally(path);
    }
    RegionFile.PathFileData data = RegionFile.read(path.name);
    if (data.getRegions().isEmpty()) {
      return followPathNormally(path);
    }
    List<Region> regions = data.getRegions();
    if (isMirrored(path, data.getWaypoints())) {
      regions = mirrorRegions(regions);
    }

    // Work out each divert path's trajectory now, while the auto is built, so the robot doesn't
    // have to stop and think in the middle of the auto
    for (Region region : regions) {
      if (region.getDivertPath() != null) {
        region.getDivertPath().getIdealTrajectory(robotConfig);
      }
    }
    return new ReactivePathCommand(path, regions);
  }

  /**
   * An auto can mirror a path to the other side of the field. Then every waypoint is at the mirror
   * of where the file says it is.
   *
   * @param waypointsInFile where each waypoint is in the file
   */
  static boolean isMirrored(PathPlannerPath path, List<Translation2d> waypointsInFile) {
    if (waypointsInFile == null || waypointsInFile.size() != path.getWaypoints().size()) {
      return false;
    }
    boolean allSame = true;
    boolean allMirrored = true;
    for (int i = 0; i < waypointsInFile.size(); i++) {
      Translation2d inPath = path.getWaypoints().get(i).anchor();
      Translation2d inFile = waypointsInFile.get(i);
      if (inPath.getDistance(inFile) > 1e-6) {
        allSame = false;
      }
      if (inPath.getDistance(PathMath.mirror(inFile)) > 1e-6) {
        allMirrored = false;
      }
    }
    // A path drawn exactly on the middle line is the same either way: treat it as not mirrored
    return allMirrored && !allSame;
  }

  /** The same regions, with zones and divert paths mirrored to the other side of the field. */
  static List<Region> mirrorRegions(List<Region> regions) {
    List<Region> mirrored = new ArrayList<>();
    for (Region region : regions) {
      List<Translation2d> zone = new ArrayList<>();
      for (Translation2d corner : region.getPickupZone()) {
        zone.add(PathMath.mirror(corner));
      }
      PathPlannerPath divert = region.getDivertPath();
      if (divert != null) {
        divert = divert.mirrorPath();
      }
      mirrored.add(
          new Region(
              region.getStartPos(),
              region.getEndPos(),
              region.getTrigger(),
              region.getTeam(),
              region.getWaitSeconds(),
              region.getAction(),
              divert,
              zone,
              region.getMaxPieceDistance()));
    }
    return mirrored;
  }

  static void checkConfigured() {
    if (robotConfig == null) {
      throw new IllegalStateException(
          "Call ReactivePathing.configure(...) in RobotContainer before building any autos.");
    }
  }

  /** Cameras with no classes set see everything as OTHER, so they never see robots or pieces. */
  private static void warnIfNoClasses(Camera camera, String method) {
    if (camera != null && !camera.hasClassesSet()) {
      DriverStation.reportWarning(
          "Reactive Pathing: the camera given to "
              + method
              + "(...) has no classes set, so it will never see robots or game pieces. Add"
              + " robotClasses(...), redRobotClasses(...), blueRobotClasses(...) or"
              + " gamePieceClasses(...) to it.",
          false);
    }
  }

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

  /** Stop the drivetrain */
  static void stopDriving() {
    driveOutput.accept(new ChassisSpeeds(), DriveFeedforwards.zeros(robotConfig.numModules));
  }

  static Set<Subsystem> driveSet() {
    Set<Subsystem> set = new HashSet<>();
    for (Subsystem subsystem : driveSubsystems) {
      set.add(subsystem);
    }
    return set;
  }

  // Getters for the path command

  static Supplier<Pose2d> getRobotPose() {
    return robotPose;
  }

  static boolean shouldFlip() {
    return shouldFlip.getAsBoolean();
  }

  static boolean isHolonomic() {
    return robotConfig.isHolonomic;
  }

  static Subsystem[] getDriveSubsystems() {
    return driveSubsystems;
  }

  static Camera getRobotCamera() {
    return robotCamera;
  }

  static Camera getGamePieceCamera() {
    return gamePieceCamera;
  }

  static Command makeIntakeCommand() {
    Command command = intakeCommand.get();
    if (command == null) {
      return Commands.none();
    }
    return command;
  }

  static BooleanSupplier getHasGamePiece() {
    return hasGamePiece;
  }

  static Subsystem[] getIntakeSubsystems() {
    return intakeSubsystems;
  }

  static double getIntakeFacingDegrees() {
    return intakeFacingDegrees;
  }

  static double getIntakeAfterArrivingSeconds() {
    return intakeAfterArrivingSeconds;
  }

  static double getMaxPickupSeconds() {
    return maxPickupSeconds;
  }

  static boolean divertAllowed() {
    return allowDivert == null || allowDivert.getAsBoolean();
  }

  static boolean waitAllowed() {
    return allowWait == null || allowWait.getAsBoolean();
  }

  static boolean pickupsAllowed() {
    return allowPickups == null || allowPickups.getAsBoolean();
  }

  /** @return the divert path to use for a region: the override if there is one */
  static PathPlannerPath divertPathFor(Region region) {
    PathPlannerPath picked = null;
    if (divertOverride != null) {
      picked = divertOverride.get();
    }
    if (picked != null) {
      return picked;
    }
    return region.getDivertPath();
  }

  /** Forget all settings, for tests */
  static void reset() {
    robotPose = null;
    robotSpeeds = null;
    driveOutput = null;
    controller = null;
    robotConfig = null;
    shouldFlip = () -> false;
    driveSubsystems = new Subsystem[0];
    robotCamera = null;
    gamePieceCamera = null;
    intakeCommand = () -> Commands.none();
    hasGamePiece = null;
    intakeSubsystems = new Subsystem[0];
    intakeFacingDegrees = 0.0;
    allowDivert = () -> true;
    allowWait = () -> true;
    allowPickups = () -> true;
    divertOverride = () -> null;
    AutoBuilder.resetForTesting();
  }
}
