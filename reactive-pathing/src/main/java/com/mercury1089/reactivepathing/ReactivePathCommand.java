package com.mercury1089.reactivepathing;

import com.mercury1089.detection.Sighting;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.FlippingUtil;
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

/**
 * Follows a path that has regions, reacting to what the cameras see. ReactivePathing makes one of
 * these for each path with regions in an auto.
 *
 * <p>It works in steps:
 *
 * <ol>
 *   <li>FOLLOWING_PATH: follow the path, checking each region every loop.
 *   <li>WAITING_FOR_ROBOT: a robot blocked a region that has a wait time. Stand still until the
 *       wait is over, then go on (or divert, if the robot is still there).
 *   <li>PICKING_UP: a game piece is in a region's zone. Drive to it with the intake running.
 *   <li>GOING_BACK_TO_PATH: after a pickup, drive back to where the region ends, then follow the
 *       rest of the path.
 *   <li>DIVERTING: follow the region's divert path instead of the rest of the path.
 *   <li>FINISHED
 * </ol>
 *
 * <p>Each step has its own command (a path follower or a pathfinder). This command runs it by hand,
 * calling its initialize(), execute(), isFinished() and end(). That way the auto sees one command
 * from start to end, and nothing else can take the drivetrain in between.
 *
 * <p>When the robot leaves the path and comes back, it follows the rest of the path from where it
 * joins. Event markers on the part it skipped don't run.
 */
public class ReactivePathCommand extends Command {
  /** A robot this close to the region's part of the path is "in the way", in meters */
  static final double BLOCKING_DISTANCE = 0.6;

  /** How far ahead along the path to look for the robot each loop, in meters */
  static final double SEARCH_AHEAD_METERS = 1.0;

  /** After reaching a game piece, keep the intake running this long (if there's no hasGamePiece) */
  static final double HOLD_STILL_SECONDS = 0.5;

  /** Give up on a pickup after this long */
  static final double MAX_PICKUP_SECONDS = 3.0;

  private enum Step {
    FOLLOWING_PATH,
    WAITING_FOR_ROBOT,
    PICKING_UP,
    GOING_BACK_TO_PATH,
    DIVERTING,
    FINISHED
  }

  /** One region, and what has happened with it so far */
  private static class RegionState {
    final Region region;
    boolean done = false; // it reacted already, or the robot drove past it
    boolean robotSeen = false; // robot regions: a robot was in the way while we were in the region

    RegionState(Region region) {
      this.region = region;
    }
  }

  private final PathPlannerPath path;
  private final List<Region> regions;
  private final List<RegionState> regionStates = new ArrayList<>();

  // The path's points, as drawn (on the blue side): where each one is, where it is along the path
  // (waypoint relative), and how far along the path it is in meters
  private final List<Translation2d> points = new ArrayList<>();
  private final List<Double> pointPositions = new ArrayList<>();
  private final List<Double> pointDistances = new ArrayList<>();

  private Step step;
  private Command current; // the command for the current step
  private boolean currentFinished;
  private int lastPointIndex; // the path point the robot was closest to last time
  private RegionState activeRegion; // the region we are waiting at or picking up for
  private double waitStartTime;
  private double rejoinPos; // where the robot joins the path again after a pickup
  private boolean finishAfterGoingBack;
  private Command intake; // the intake command we started, or null
  private boolean intakeRunByHand; // true if we run the intake command ourselves

  public ReactivePathCommand(PathPlannerPath path, List<Region> regions) {
    this.path = path;
    this.regions = regions;
    for (PathPoint point : path.getAllPathPoints()) {
      points.add(point.position);
      pointPositions.add(point.waypointRelativePos);
      pointDistances.add(point.distanceAlongPath);
    }

    // We run the path follower and its event markers ourselves, so we need their subsystems.
    // We don't need the intake: its command is started separately (see startIntake).
    addRequirements(ReactivePathing.driveSubsystems);
    addRequirements(EventScheduler.getSchedulerRequirements(path));
    for (Region region : regions) {
      if (region.divertPath != null) {
        addRequirements(EventScheduler.getSchedulerRequirements(region.divertPath));
      }
    }
  }

  // ---------------------------------------------------------------- the command

  @Override
  public void initialize() {
    current = null;
    currentFinished = true;
    activeRegion = null;
    intake = null;
    finishAfterGoingBack = false;
    regionStates.clear();
    for (Region region : regions) {
      regionStates.add(new RegionState(region));
    }
    startFollowing(0.0);
  }

  @Override
  public void execute() {
    if (step == Step.FOLLOWING_PATH) {
      if (checkRegions()) {
        return; // a region reacted and started a new step
      }
      runCurrent();
      if (currentFinished) {
        // The path is done. A region that ends right at the end of the path gets one last check.
        if (!checkRegions()) {
          setStep(Step.FINISHED);
        }
      }

    } else if (step == Step.WAITING_FOR_ROBOT) {
      ReactivePathing.stopDriving();
      Region region = activeRegion.region;
      double waited = Timer.getFPGATimestamp() - waitStartTime;
      if (waited < region.waitSeconds) {
        return; // keep waiting
      }
      // Done waiting. If the robot is still there, divert (if the region says to).
      boolean stillBlocked = isBlocked(region);
      if (stillBlocked && region.action == Region.Action.DIVERT && startDiverting(region)) {
        return;
      }
      startFollowing(currentPathPosition());

    } else if (step == Step.PICKING_UP) {
      runCurrent();
      runIntakeByHand();
      if (currentFinished) {
        stopIntake();
        Region region = activeRegion.region;
        if (region.action == Region.Action.DIVERT && startDiverting(region)) {
          return;
        }
        startGoingBack(region);
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

  // ---------------------------------------------------------------- checking regions

  /**
   * Check every region that hasn't reacted yet.
   *
   * @return true if one reacted (and started a new step)
   */
  private boolean checkRegions() {
    double pos = currentPathPosition();
    for (RegionState state : regionStates) {
      if (state.done) {
        continue;
      }
      boolean reacted;
      if (state.region.trigger == Region.Trigger.GAME_PIECE) {
        reacted = checkGamePieceRegion(state, pos);
      } else {
        reacted = checkRobotRegion(state, pos);
      }
      if (reacted) {
        return true;
      }
    }
    return false;
  }

  /**
   * A "robot in the way" region. A robot seen in the region is remembered. At the end of the
   * region: wait (if it has a wait time), or divert (if it says to), or just keep going.
   */
  private boolean checkRobotRegion(RegionState state, double pos) {
    Region region = state.region;
    if (ReactivePathing.robotCamera == null) {
      return false;
    }
    if (!state.robotSeen) {
      state.robotSeen = region.contains(pos) && isBlocked(region);
    }
    // ">=" so a region that ends right at the end of the path still gets here
    boolean atRegionEnd = pos >= region.endPos;
    if (!state.robotSeen || !atRegionEnd) {
      return false;
    }

    state.done = true;
    if (region.waitSeconds > 0) {
      stopCurrent();
      activeRegion = state;
      waitStartTime = Timer.getFPGATimestamp();
      setStep(Step.WAITING_FOR_ROBOT);
      return true;
    }
    if (region.action == Region.Action.DIVERT) {
      return startDiverting(region);
    }
    return false;
  }

  /** A game piece region: if a piece is in the zone while we're in the region, go get it. */
  private boolean checkGamePieceRegion(RegionState state, double pos) {
    Region region = state.region;
    if (pos > region.endPos) {
      state.done = true; // drove through without seeing a piece
      return false;
    }
    if (!region.contains(pos) || !canPickUp()) {
      return false;
    }
    Translation2d piece = findGamePiece(region);
    if (piece == null) {
      return false;
    }
    state.done = true;
    activeRegion = state;
    startPickup(piece);
    return true;
  }

  /** @return true if a robot that counts for this region is near the region's part of the path */
  private boolean isBlocked(Region region) {
    for (Sighting robot : ReactivePathing.robotCamera.getRobots()) {
      Translation2d robotPosition = robot.getFieldPosition();
      if (robotPosition == null || !teamCounts(region.team, robot.getColor())) {
        continue;
      }
      for (int i = 0; i < points.size(); i++) {
        boolean inRegion = region.contains(pointPositions.get(i));
        if (inRegion && onField(points.get(i)).getDistance(robotPosition) <= BLOCKING_DISTANCE) {
          return true;
        }
      }
    }
    return false;
  }

  /** @return true if a robot of this color counts for a region set to this team */
  static boolean teamCounts(Region.Team team, Sighting.Color robotColor) {
    if (team == Region.Team.ANY) {
      return true;
    }
    Sighting.Color ourColor = Sighting.Color.UNKNOWN;
    if (DriverStation.getAlliance().isPresent()) {
      boolean red = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
      ourColor = red ? Sighting.Color.RED : Sighting.Color.BLUE;
    }
    if (ourColor == Sighting.Color.UNKNOWN || robotColor == Sighting.Color.UNKNOWN) {
      return false; // can't tell, so it doesn't count
    }
    if (team == Region.Team.TEAMMATE) {
      return robotColor == ourColor;
    }
    return robotColor != ourColor;
  }

  private boolean canPickUp() {
    if (ReactivePathing.gamePieceCamera == null) {
      return false;
    }
    // Already holding a piece: don't go get another
    return ReactivePathing.hasGamePiece == null || !ReactivePathing.hasGamePiece.getAsBoolean();
  }

  /** @return the closest game piece inside the region's zone, or null if there is none */
  private Translation2d findGamePiece(Region region) {
    Translation2d robot = ReactivePathing.robotPose.get().getTranslation();

    // The zone is drawn on the blue side, like paths
    List<Translation2d> zone = new ArrayList<>();
    for (Translation2d corner : region.pickupZone) {
      zone.add(onField(corner));
    }

    Translation2d best = null;
    for (Sighting piece : ReactivePathing.gamePieceCamera.getGamePieces()) {
      Translation2d at = piece.getFieldPosition();
      if (at == null) {
        continue;
      }
      boolean inZone;
      if (zone.size() >= 3) {
        inZone = PathTools.isInside(zone, at);
      } else {
        inZone = at.getDistance(robot) <= Region.NO_ZONE_PIECE_DISTANCE; // old regions with no zone
      }
      if (inZone && (best == null || at.getDistance(robot) < best.getDistance(robot))) {
        best = at;
      }
    }
    return best;
  }

  // ---------------------------------------------------------------- starting each step

  /** Follow the path from a position along it to the end. */
  private void startFollowing(double fromPos) {
    PathPlannerPath rest = PathTools.restOfPath(path, fromPos);
    if (rest == null) {
      stopCurrent();
      setStep(Step.FINISHED);
      return;
    }
    lastPointIndex = firstPointAtOrAfter(fromPos);
    startCommand(ReactivePathing.followPath(rest));
    setStep(Step.FOLLOWING_PATH);
  }

  /** Drive to a game piece with the intake running. */
  private void startPickup(Translation2d piece) {
    // Face the piece with the side of the robot that has the intake
    Pose2d robot = ReactivePathing.robotPose.get();
    Rotation2d heading = piece.minus(robot.getTranslation()).getAngle();
    if (ReactivePathing.intakeOnBack) {
      heading = heading.plus(Rotation2d.k180deg);
    }

    // Drive to the piece (a real field position, so it isn't flipped), then hold still a moment
    Command pickup =
        Commands.sequence(
            ReactivePathing.pathfindToPose(new Pose2d(piece, heading), path.getGlobalConstraints()),
            Commands.run(ReactivePathing::stopDriving).withTimeout(HOLD_STILL_SECONDS));
    // Stop as soon as the piece is in, and never take too long
    if (ReactivePathing.hasGamePiece != null) {
      pickup = pickup.until(ReactivePathing.hasGamePiece);
    }
    pickup = pickup.withTimeout(MAX_PICKUP_SECONDS);

    ReactivePathing.stopDriving(); // the pathfinder takes a moment to start: don't coast meanwhile
    startCommand(pickup);
    startIntake();
    setStep(Step.PICKING_UP);
  }

  /**
   * Drive back to the path where the region ends, facing the way the path faces there, then follow
   * the rest of the path. It arrives stopped, so it doesn't swerve onto the path.
   */
  private void startGoingBack(Region region) {
    int join = firstPointAtOrAfter(region.endPos);
    boolean atTheEnd = join >= points.size() - 1;

    Pose2d target;
    if (atTheEnd) {
      // The region ends at the end of the path: just go to the end
      target = new Pose2d(points.get(points.size() - 1), path.getGoalEndState().rotation());
      finishAfterGoingBack = true;
    } else {
      Translation2d joinPoint = points.get(join);
      // Face the way the path goes there (backwards, if the path is driven in reverse)...
      Rotation2d heading = points.get(join + 1).minus(joinPoint).getAngle();
      if (path.isReversed()) {
        heading = heading.plus(Rotation2d.k180deg);
      }
      // ...or, for swerve, the way the path says to face
      if (ReactivePathing.robotConfig.isHolonomic) {
        Rotation2d pathRotation = PathTools.rotationAt(path, region.endPos);
        if (pathRotation != null) {
          heading = pathRotation;
        }
      }
      target = new Pose2d(joinPoint, heading);
      rejoinPos = pointPositions.get(join);
      finishAfterGoingBack = false;
    }

    ReactivePathing.stopDriving(); // the pathfinder takes a moment to start: don't coast meanwhile
    startCommand(ReactivePathing.pathfindToPoseFlipped(target, path.getGlobalConstraints()));
    setStep(Step.GOING_BACK_TO_PATH);
  }

  /**
   * Follow the region's divert path, from wherever the robot is.
   *
   * @return false if the region has no divert path
   */
  private boolean startDiverting(Region region) {
    PathPlannerPath divert = region.divertPath;
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

  // ---------------------------------------------------------------- the intake

  /**
   * Start the intake command for a pickup. It is never made part of this command (that would clash
   * with autos that use the intake too). For each subsystem it needs:
   *
   * <ul>
   *   <li>Free (or only running its default command): we schedule the intake command.
   *   <li>Held by our own auto (an auto holds every subsystem its commands use): we run the intake
   *       command by hand, like the steps.
   *   <li>Held by some other command: we don't start the intake, so that command isn't cancelled.
   * </ul>
   */
  private void startIntake() {
    intake = null;
    if (ReactivePathing.intakeCommand == null) {
      return;
    }
    Command command = ReactivePathing.intakeCommand.get();
    if (command == null) {
      return;
    }
    CommandScheduler scheduler = CommandScheduler.getInstance();
    if (scheduler.isComposed(command)) {
      DriverStation.reportWarning(
          "Reactive Pathing: can't start the intake. Give pickUpGamePieces(...) a method that makes"
              + " a new command each time, such as intake::intakeCommand.",
          false);
      return;
    }

    Command ourAuto = null; // whatever is running this command (it holds the drivetrain)
    if (ReactivePathing.driveSubsystems.length > 0) {
      ourAuto = scheduler.requiring(ReactivePathing.driveSubsystems[0]);
    }
    boolean ourAutoHasIt = false;
    for (Subsystem subsystem : command.getRequirements()) {
      Command using = scheduler.requiring(subsystem);
      if (using == null || using == subsystem.getDefaultCommand()) {
        continue; // free
      }
      if (using == ourAuto) {
        ourAutoHasIt = true;
        continue;
      }
      DriverStation.reportWarning(
          "Reactive Pathing: didn't start the intake, because " + using.getName() + " is using it",
          false);
      return;
    }

    intake = command;
    intakeRunByHand = ourAutoHasIt;
    if (intakeRunByHand) {
      intake.initialize();
    } else {
      scheduler.schedule(intake);
    }
  }

  /** Run the intake command for one loop, if we are running it by hand. */
  private void runIntakeByHand() {
    if (intake == null || !intakeRunByHand) {
      return;
    }
    intake.execute();
    if (intake.isFinished()) {
      intake.end(false);
      intake = null;
    }
  }

  /** Stop the intake command we started, if it is still running. */
  private void stopIntake() {
    if (intake == null) {
      return;
    }
    if (intakeRunByHand) {
      intake.end(true);
    } else {
      CommandScheduler.getInstance().cancel(intake);
    }
    intake = null;
  }

  // ---------------------------------------------------------------- running a step's command

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

  /** End the current command early, if it is still running. */
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

  // ---------------------------------------------------------------- where the robot is

  /**
   * Where the robot is along the path (waypoint relative). It only looks a little ahead of where
   * the robot was last time, and never goes backwards, so a path that comes back near itself
   * doesn't confuse it.
   */
  private double currentPathPosition() {
    Translation2d robot = ReactivePathing.robotPose.get().getTranslation();
    double lastDistance = pointDistances.get(lastPointIndex);
    double bestDistance = Double.POSITIVE_INFINITY;
    for (int i = lastPointIndex; i < points.size(); i++) {
      if (pointDistances.get(i) > lastDistance + SEARCH_AHEAD_METERS) {
        break;
      }
      double distance = onField(points.get(i)).getDistance(robot);
      if (distance < bestDistance) {
        bestDistance = distance;
        lastPointIndex = i;
      }
    }
    return pointPositions.get(lastPointIndex);
  }

  /** @return the index of the first path point at or after a position (or the last point) */
  private int firstPointAtOrAfter(double pos) {
    for (int i = 0; i < pointPositions.size(); i++) {
      if (pointPositions.get(i) >= pos) {
        return i;
      }
    }
    return pointPositions.size() - 1;
  }

  /** A position drawn on the blue side, on the real field (flipped when we are red). */
  private static Translation2d onField(Translation2d bluePosition) {
    if (ReactivePathing.shouldFlip.getAsBoolean()) {
      return FlippingUtil.flipFieldPosition(bluePosition);
    }
    return bluePosition;
  }
}
