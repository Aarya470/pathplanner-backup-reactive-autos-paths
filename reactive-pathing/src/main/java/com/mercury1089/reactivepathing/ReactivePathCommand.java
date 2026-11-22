package com.mercury1089.reactivepathing;

import com.mercury1089.vision.Camera;
import com.mercury1089.vision.Sighting;
import com.mercury1089.vision.TeamColor;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

/**
 * Follows a path that has regions, reacting to what the cameras see. Made for you by
 * ReactivePathing when an auto uses a path with regions.
 *
 * <p>It works in steps:
 *
 * <ol>
 *   <li>FOLLOWING_PATH: follow the path, checking each region every loop.
 *   <li>WAITING_FOR_ROBOT: a robot blocked a region that has a wait time. Stand still until the
 *       wait is over, then go on, or divert if the robot is still there.
 *   <li>PICKING_UP: a game piece is in a region's zone. Drive to it with the intake running.
 *   <li>GOING_BACK_TO_PATH: after a pickup, drive back to the path where the region ends, then
 *       follow the rest of the path.
 *   <li>DIVERTING: follow the region's divert path instead of the rest of the main path.
 *   <li>FINISHED
 * </ol>
 *
 * <p>Each step has its own command (a path follower, or a pathfinder). This command runs that
 * command itself, by calling its initialize(), execute(), isFinished() and end() each loop, instead
 * of scheduling it. That way the auto sees one command from start to end, and nothing else can
 * take the drivetrain in between.
 *
 * <p>When the robot stops or leaves the path and comes back, the rest of the path is followed from
 * where it joins. Event markers on the part that was skipped don't run, and marker commands still
 * running when the robot stopped (such as one started with a zone) are ended. Zones that cover the
 * join point start again.
 */
public class ReactivePathCommand extends Command {
  /** How close a robot must be to the region's part of the path to count as "in the way" */
  static final double BLOCKING_DISTANCE = 0.6;

  /**
   * How far ahead along the path to look for the robot each loop, in meters. Looking only a little
   * ahead keeps out-and-back paths from jumping to the way back.
   */
  static final double SEARCH_AHEAD_METERS = 1.0;

  /** One region, and what has happened with it in this run */
  private static class RegionState {
    final Region region;
    final RegionAction action; // the region's action, or CONTINUE if diverting is switched off
    final RobotRegionWatcher watcher; // only used by "robot in the way" regions
    boolean done = false; // true once the region has reacted, or the robot drove past it

    RegionState(Region region, RegionAction action, RobotRegionWatcher watcher) {
      this.region = region;
      this.action = action;
      this.watcher = watcher;
    }
  }

  private enum Step {
    FOLLOWING_PATH,
    WAITING_FOR_ROBOT,
    PICKING_UP,
    GOING_BACK_TO_PATH,
    DIVERTING,
    FINISHED
  }

  private final PathPlannerPath path;
  private final List<Region> regions;

  // The path's points, drawn on the blue side, where each one is along the path (waypoint
  // relative), and how far along the path each one is in meters
  private final List<Translation2d> bluePoints = new ArrayList<>();
  private final List<Double> pointPositions = new ArrayList<>();
  private final List<Double> pointDistances = new ArrayList<>();

  // What is happening in this run
  private Step step;
  private Command current; // the command for the current step
  private boolean currentFinished;
  private int lastIndex; // the path point the robot was at, last time we looked
  private double rejoinPos; // where the robot joins the path again after a pickup
  private boolean finishAfterGoingBack;
  private RegionState activeRegion; // the region that started the current step
  private Command runningIntake; // the intake command we started, or null
  private boolean intakeRunByHand; // true if we run runningIntake ourselves, like the steps
  private final List<RegionState> regionStates = new ArrayList<>();

  /**
   * @param path the path, as the auto gives it (it may be mirrored)
   * @param regions the path's regions, already mirrored if the path is
   */
  public ReactivePathCommand(PathPlannerPath path, List<Region> regions) {
    this.path = path;
    this.regions = regions;

    for (PathPoint point : path.getAllPathPoints()) {
      bluePoints.add(point.position);
      pointPositions.add(point.waypointRelativePos);
      pointDistances.add(point.distanceAlongPath);
    }

    // This command runs the path follower and the event markers itself, so it needs all of their
    // subsystems. It doesn't need the intake: the intake command is started on its own (see
    // startIntake), so autos can still run the intake alongside this path.
    addRequirements(ReactivePathing.getDriveSubsystems());
    addRequirements(EventScheduler.getSchedulerRequirements(path));
    for (Region region : regions) {
      PathPlannerPath divert = ReactivePathing.divertPathFor(region);
      if (divert != null) {
        addRequirements(EventScheduler.getSchedulerRequirements(divert));
      }
    }
  }

  // ------------------------------------------------------------------ the command

  @Override
  public void initialize() {
    current = null;
    currentFinished = true;
    finishAfterGoingBack = false;
    activeRegion = null;
    runningIntake = null;

    // Fresh rules for each region. Switches are read now, when the path starts.
    regionStates.clear();
    for (Region region : regions) {
      double waitSeconds = region.getWaitSeconds();
      if (!ReactivePathing.waitAllowed()) {
        waitSeconds = 0;
      }
      RegionAction action = region.getAction();
      if (!ReactivePathing.divertAllowed()) {
        action = RegionAction.CONTINUE;
      }
      regionStates.add(
          new RegionState(region, action, new RobotRegionWatcher(waitSeconds, action)));
    }

    startFollowing(0.0);
  }

  @Override
  public void execute() {
    if (step == Step.FOLLOWING_PATH) {
      boolean reacted = checkRegions();
      if (reacted) {
        return;
      }
      runCurrent();
      if (currentFinished) {
        // The path is done. A region that ends at the very end of the path gets one last check.
        boolean reactedAtTheEnd = checkRegions();
        if (!reactedAtTheEnd) {
          setStep(Step.FINISHED);
        }
      }
    } else if (step == Step.WAITING_FOR_ROBOT) {
      ReactivePathing.stopDriving();
      double pos = currentPathPosition();
      RobotRegionWatcher.Decision decision = updateWatcher(activeRegion, pos);
      if (decision == RobotRegionWatcher.Decision.RESUME) {
        startFollowing(pos);
      } else if (decision == RobotRegionWatcher.Decision.DIVERT) {
        if (!startDiverting(activeRegion.region)) {
          startFollowing(pos);
        }
      }
    } else if (step == Step.PICKING_UP) {
      runCurrent();
      runIntakeByHand();
      if (currentFinished) {
        stopIntake();
        afterPickup(activeRegion);
      }
    } else if (step == Step.GOING_BACK_TO_PATH) {
      runCurrent();
      if (currentFinished) {
        if (finishAfterGoingBack) {
          setStep(Step.FINISHED);
        } else {
          startFollowing(rejoinPos);
        }
      }
    } else if (step == Step.DIVERTING) {
      runCurrent();
      if (currentFinished) {
        setStep(Step.FINISHED);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return step == Step.FINISHED;
  }

  @Override
  public void end(boolean interrupted) {
    stopCurrent();
    stopIntake();
    if (interrupted) {
      ReactivePathing.stopDriving();
    }
  }

  // ------------------------------------------------------------------ starting each step

  /** Follow the path from a position along it to the end. */
  private void startFollowing(double fromPos) {
    PathPlannerPath rest = PathCutter.restOfPath(path, fromPos);
    if (rest == null) {
      stopCurrent();
      setStep(Step.FINISHED);
      return;
    }
    lastIndex = firstPointAtOrAfter(fromPos);
    if (lastIndex < 0) {
      lastIndex = pointPositions.size() - 1;
    }
    startCommand(ReactivePathing.followPathNormally(rest));
    setStep(Step.FOLLOWING_PATH);
  }

  /** Drive to a game piece with the intake running. */
  private void startPickup(RegionState state, Translation2d piece) {
    Pose2d robot = ReactivePathing.getRobotPose().get();

    // Face the piece with the side of the robot that has the intake
    Rotation2d towardPiece = piece.minus(robot.getTranslation()).getAngle();
    Rotation2d heading = towardPiece.plus(Rotation2d.fromDegrees(ReactivePathing.getIntakeFacingDegrees()));

    // Step 1: drive to the piece. It is a real field position, so it is not flipped.
    Command driveToPiece =
        ReactivePathing.pathfindToPose(new Pose2d(piece, heading), path.getGlobalConstraints());

    // Step 2: once there, stand still for a moment so the intake can grab it
    Command holdStill =
        Commands.run(() -> ReactivePathing.stopDriving())
            .withTimeout(ReactivePathing.getIntakeAfterArrivingSeconds());
    Command pickup = Commands.sequence(driveToPiece, holdStill);

    // Stop early as soon as the piece is in
    BooleanSupplier hasPiece = ReactivePathing.getHasGamePiece();
    if (hasPiece != null) {
      pickup = pickup.until(hasPiece);
    }

    // Never take longer than this, even if the piece is never picked up
    pickup = pickup.withTimeout(ReactivePathing.getMaxPickupSeconds());

    activeRegion = state;
    ReactivePathing.stopDriving(); // the pathfinder takes a moment to start: don't coast meanwhile
    startCommand(pickup);
    startIntake();
    setStep(Step.PICKING_UP);
  }

  /**
   * Start the team's intake command for a pickup. It is never made part of this command (that
   * would clash with autos that use the intake too). Instead, for each subsystem it needs:
   *
   * <ul>
   *   <li>Free (or only running its default command): fine.
   *   <li>Used by the auto this path is part of (the auto needs every subsystem any of its commands
   *       use, so this is common): fine, but the auto owns it, so we run the intake command
   *       ourselves, like the steps.
   *   <li>Used by some other command: don't start the intake, because starting it would cancel
   *       that command, which could even be the whole auto.
   * </ul>
   *
   * If every subsystem is free, the intake command is scheduled normally.
   */
  private void startIntake() {
    runningIntake = null;
    intakeRunByHand = false;
    CommandScheduler scheduler = CommandScheduler.getInstance();

    Command intakeCommand = ReactivePathing.makeIntakeCommand();
    if (scheduler.isComposed(intakeCommand)) {
      DriverStation.reportWarning(
          "Reactive Pathing: the intake command is also used inside another command group, so it"
              + " can't be started. Give pickUpGamePieces(...) a method that makes a new command"
              + " each time, such as intake::intakeCommand.",
          false);
      return;
    }

    // Our auto: whatever is running this command (it holds the drivetrain)
    Command ourAuto = null;
    Subsystem[] drive = ReactivePathing.getDriveSubsystems();
    if (drive.length > 0) {
      ourAuto = scheduler.requiring(drive[0]);
    }

    boolean ourAutoHasIt = false;
    for (Subsystem subsystem : intakeCommand.getRequirements()) {
      Command using = scheduler.requiring(subsystem);
      if (using == null || using == subsystem.getDefaultCommand()) {
        continue; // free
      }
      if (using == ourAuto) {
        ourAutoHasIt = true;
        continue;
      }
      DriverStation.reportWarning(
          "Reactive Pathing: didn't start the intake for a pickup, because "
              + using.getName()
              + " is using "
              + subsystem.getName(),
          false);
      return;
    }

    runningIntake = intakeCommand;
    if (ourAutoHasIt) {
      intakeRunByHand = true;
      intakeCommand.initialize();
    } else {
      scheduler.schedule(intakeCommand);
    }
  }

  /** Run the intake command for one loop, if we are running it ourselves. */
  private void runIntakeByHand() {
    if (runningIntake == null || !intakeRunByHand) {
      return;
    }
    runningIntake.execute();
    if (runningIntake.isFinished()) {
      runningIntake.end(false);
      runningIntake = null;
    }
  }

  /** Stop the intake command we started, if it is still running. */
  private void stopIntake() {
    if (runningIntake == null) {
      return;
    }
    if (intakeRunByHand) {
      runningIntake.end(true);
    } else {
      CommandScheduler.getInstance().cancel(runningIntake);
    }
    runningIntake = null;
  }

  /** After a pickup: divert, or go back to the path. */
  private void afterPickup(RegionState state) {
    if (state.action == RegionAction.DIVERT && startDiverting(state.region)) {
      return;
    }
    startGoingBack(state.region);
  }

  /**
   * Drive back to the path where the region ends, facing the way the path faces there. Then follow
   * the rest of the path. It arrives stopped: the pathfinder could arrive from any direction, so
   * arriving at speed could make the robot swerve onto the path.
   */
  private void startGoingBack(Region region) {
    int join = firstPointAtOrAfter(region.getEndPos());
    boolean nothingLeft = join < 0 || join >= bluePoints.size() - 1;
    PathConstraints constraints = path.getGlobalConstraints();
    activeRegion = null;
    ReactivePathing.stopDriving(); // the pathfinder takes a moment to start: don't coast meanwhile

    if (nothingLeft) {
      // The region ends at the end of the path: just go to the end
      Translation2d end = bluePoints.get(bluePoints.size() - 1);
      Pose2d endPose = new Pose2d(end, path.getGoalEndState().rotation());
      finishAfterGoingBack = true;
      startCommand(ReactivePathing.pathfindToPoseFlipped(endPose, constraints, 0.0));
      setStep(Step.GOING_BACK_TO_PATH);
      return;
    }

    Translation2d joinPoint = bluePoints.get(join);
    Translation2d nextPoint = bluePoints.get(join + 1);
    Rotation2d travelDirection = nextPoint.minus(joinPoint).getAngle();

    // Holonomic robots face the way the path faces there. Others face the way the path goes
    // (backwards, if the path is driven in reverse).
    Rotation2d heading = travelDirection;
    if (!ReactivePathing.isHolonomic() && path.isReversed()) {
      heading = travelDirection.plus(Rotation2d.k180deg);
    }
    if (ReactivePathing.isHolonomic()) {
      Rotation2d pathRotation = PathMath.rotationAt(path, region.getEndPos());
      if (pathRotation != null) {
        heading = pathRotation;
      }
    }

    // Follow the rest of the path from exactly the point we drive to
    rejoinPos = pointPositions.get(join);
    finishAfterGoingBack = false;
    startCommand(
        ReactivePathing.pathfindToPoseFlipped(new Pose2d(joinPoint, heading), constraints, 0.0));
    setStep(Step.GOING_BACK_TO_PATH);
  }

  /**
   * Follow the region's divert path, from wherever the robot is.
   *
   * @return false if the region has no divert path
   */
  private boolean startDiverting(Region region) {
    PathPlannerPath divert = ReactivePathing.divertPathFor(region);
    if (divert == null) {
      DriverStation.reportWarning(
          "Reactive Pathing: a region on " + path.name + " is set to divert but has no divert path",
          false);
      return false;
    }
    ReactivePathing.stopDriving(); // the pathfinder takes a moment to start: don't coast meanwhile
    startCommand(ReactivePathing.pathfindThenFollowPath(divert, divert.getGlobalConstraints()));
    setStep(Step.DIVERTING);
    return true;
  }

  // ------------------------------------------------------------------ checking regions

  /**
   * Check every region that isn't done yet.
   *
   * @return true if a region reacted (and started a new step)
   */
  private boolean checkRegions() {
    double pos = currentPathPosition();

    for (RegionState state : regionStates) {
      if (state.done) {
        continue;
      }
      Region region = state.region;

      if (region.isGamePieceRegion()) {
        if (pos > region.getEndPos()) {
          state.done = true; // drove through without seeing a piece
          continue;
        }
        if (!region.contains(pos) || !canPickUp()) {
          continue;
        }
        Translation2d piece = findGamePiece(region);
        if (piece != null) {
          state.done = true;
          startPickup(state, piece);
          return true;
        }
        continue;
      }

      // A "robot in the way" region
      if (ReactivePathing.getRobotCamera() == null) {
        continue;
      }
      RobotRegionWatcher.Decision decision = updateWatcher(state, pos);
      if (state.watcher.isDone()) {
        state.done = true;
      }
      if (decision == RobotRegionWatcher.Decision.STOP) {
        stopCurrent();
        ReactivePathing.stopDriving();
        activeRegion = state;
        setStep(Step.WAITING_FOR_ROBOT);
        return true;
      }
      if (decision == RobotRegionWatcher.Decision.DIVERT) {
        if (startDiverting(region)) {
          return true;
        }
      }
    }
    return false;
  }

  /**
   * @param pos where the robot is along the path
   */
  private RobotRegionWatcher.Decision updateWatcher(RegionState state, double pos) {
    Region region = state.region;
    boolean inRegion = region.contains(pos);
    // ">=" so a region that ends exactly at the end of the path still counts as reached
    boolean pastEnd = pos >= region.getEndPos();
    boolean blocked = isBlocked(region);
    return state.watcher.update(inRegion, pastEnd, blocked, Timer.getFPGATimestamp());
  }

  /** @return true if a robot that counts for this region is near the region's part of the path */
  private boolean isBlocked(Region region) {
    Camera camera = ReactivePathing.getRobotCamera();
    if (camera == null) {
      return false;
    }
    for (Sighting robot : camera.getRobots()) {
      Translation2d robotPosition = robot.getFieldPosition();
      if (robotPosition == null || !teamCounts(region.getTeam(), robot.getColor())) {
        continue;
      }
      for (int i = 0; i < bluePoints.size(); i++) {
        if (region.contains(pointPositions.get(i))) {
          Translation2d pathPoint = fieldPosition(bluePoints.get(i));
          if (pathPoint.getDistance(robotPosition) <= BLOCKING_DISTANCE) {
            return true;
          }
        }
      }
    }
    return false;
  }

  /** @return true if a robot of this color counts for a region set to this team */
  static boolean teamCounts(RegionTeam team, TeamColor robotColor) {
    if (team == RegionTeam.ANY) {
      return true;
    }
    TeamColor ourColor = ourAlliance();
    if (ourColor == TeamColor.UNKNOWN || robotColor == TeamColor.UNKNOWN) {
      return false; // can't tell, so it doesn't count
    }
    if (team == RegionTeam.TEAMMATE) {
      return robotColor == ourColor;
    }
    return robotColor != ourColor;
  }

  private boolean canPickUp() {
    if (!ReactivePathing.pickupsAllowed() || ReactivePathing.getGamePieceCamera() == null) {
      return false;
    }
    // Already holding a piece: don't go get another
    BooleanSupplier hasPiece = ReactivePathing.getHasGamePiece();
    return hasPiece == null || !hasPiece.getAsBoolean();
  }

  /** @return the closest game piece inside the region's zone, or null if there is none */
  private Translation2d findGamePiece(Region region) {
    Translation2d robot = ReactivePathing.getRobotPose().get().getTranslation();

    // The zone is drawn on the blue side, like paths
    List<Translation2d> zone = new ArrayList<>();
    for (Translation2d corner : region.getPickupZone()) {
      zone.add(fieldPosition(corner));
    }

    Translation2d best = null;
    double bestDistance = Double.POSITIVE_INFINITY;
    for (Sighting piece : ReactivePathing.getGamePieceCamera().getGamePieces()) {
      Translation2d at = piece.getFieldPosition();
      if (at == null) {
        continue;
      }
      double distance = at.getDistance(robot);
      boolean counts;
      if (region.hasPickupZone()) {
        counts = PathMath.isInside(zone, at);
      } else {
        counts = distance <= region.getMaxPieceDistance(); // old regions with no zone
      }
      if (counts && distance < bestDistance) {
        best = at;
        bestDistance = distance;
      }
    }
    return best;
  }

  // ------------------------------------------------------------------ helpers

  /**
   * Where the robot is along the path, as a waypoint-relative position. It only looks a little
   * ahead (1 m) of where the robot was last time, and never goes backwards. Call it once per loop.
   */
  private double currentPathPosition() {
    Translation2d robot = ReactivePathing.getRobotPose().get().getTranslation();
    List<Translation2d> fieldPoints = new ArrayList<>();
    for (Translation2d point : bluePoints) {
      fieldPoints.add(fieldPosition(point));
    }
    double lastDistance = pointDistances.get(lastIndex);
    int index =
        PathMath.closestPointIndex(
            fieldPoints, pointDistances, robot, lastDistance, lastDistance + SEARCH_AHEAD_METERS);
    if (index >= 0) {
      lastIndex = index;
    }
    return pointPositions.get(lastIndex);
  }

  /** @return the index of the first path point at or after a position, or -1 */
  private int firstPointAtOrAfter(double pos) {
    for (int i = 0; i < pointPositions.size(); i++) {
      if (pointPositions.get(i) >= pos) {
        return i;
      }
    }
    return -1;
  }

  /** A position drawn on the blue side, on the real field (flipped when we are red). */
  private static Translation2d fieldPosition(Translation2d bluePosition) {
    if (ReactivePathing.shouldFlip()) {
      return PathMath.flipToRed(bluePosition);
    }
    return bluePosition;
  }

  private static TeamColor ourAlliance() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return TeamColor.UNKNOWN;
    }
    if (alliance.get() == DriverStation.Alliance.Red) {
      return TeamColor.RED;
    }
    return TeamColor.BLUE;
  }

  /** Start a step's command, ending the one before it. */
  private void startCommand(Command command) {
    stopCurrent();
    current = command;
    currentFinished = false;
    current.initialize();
  }

  /** Run the current command for one loop. */
  private void runCurrent() {
    if (current == null || currentFinished) {
      return;
    }
    current.execute();
    if (current.isFinished()) {
      current.end(false);
      currentFinished = true;
    }
  }

  /** End the current command early, if it is still running, and stop the drivetrain. */
  private void stopCurrent() {
    if (current != null && !currentFinished) {
      current.end(true);
      ReactivePathing.stopDriving(); // PathPlanner's follower doesn't stop the robot when ended
    }
    currentFinished = true;
  }

  private void setStep(Step newStep) {
    step = newStep;
    SmartDashboard.putString("Reactive Pathing/Step", newStep.toString());
  }
}
