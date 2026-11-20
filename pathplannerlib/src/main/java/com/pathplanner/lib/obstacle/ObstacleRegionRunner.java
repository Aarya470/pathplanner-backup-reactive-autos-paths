package com.pathplanner.lib.obstacle;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.obstacle.RegionMonitor.Decision;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;

/**
 * Builds the command for a path that has regions.
 *
 * <p>The path runs as normal. Robot regions: when another robot blocks the region,
 *
 * <ul>
 *   <li>No timeout, DIVERT: leave the path at the region end and follow the diverge path.
 *   <li>Timeout: stop, wait, then either resume the main path (robot gone, or action is CONTINUE)
 *       or divert (robot still there and action is DIVERT).
 * </ul>
 *
 * <p>Game piece regions: while the robot is in the region, if a game piece is close enough, leave
 * the path, drive to the piece with the intake running, then either rejoin the main path after the
 * region (CONTINUE) or follow the diverge path (DIVERT). If no piece is seen, the main path goes
 * on, and so does a robot that is already holding a piece (when a has-piece sensor is set up).
 * After a rejoin, later regions on the same path are not watched.
 *
 * <p>Robot code does not call this directly. {@code AutoBuilder.followPath} uses it when the path
 * has regions and {@link ObstacleSettings} is configured.
 */
public final class ObstacleRegionRunner {
  /** How close an obstacle must be to the path to block the region. */
  private static final double BLOCK_RADIUS_METERS = 0.6;

  private ObstacleRegionRunner() {}

  /** A region plus its state for one run */
  private static final class Watch {
    final ObstacleRegion region;
    final RegionMonitor monitor;
    boolean pieceDone = false;
    Translation2d pieceTarget = null;

    Watch(ObstacleRegion region) {
      this.region = region;
      this.monitor =
          new RegionMonitor(
              ObstacleSettings.effectiveTimeout(region.getTimeoutSec()),
              ObstacleSettings.effectiveAction(region.getAction()));
    }

    boolean isPiece() {
      return region.getTrigger() == RegionTrigger.GAME_PIECE;
    }

    boolean isDone() {
      return isPiece() ? pieceDone : monitor.isDone();
    }
  }

  /**
   * Build the command for a path with regions. The dashboard options are read when the command
   * starts, not when it is built, so they can be changed right up until the path runs.
   *
   * @param path the main path
   * @param drive the drive subsystems the paths require
   * @return the command to run
   */
  public static Command build(PathPlannerPath path, Subsystem[] drive) {
    Set<Subsystem> reqs = new HashSet<>(Arrays.asList(drive));
    for (ObstacleRegion r : path.getObstacleRegions()) {
      if (r.getTrigger() == RegionTrigger.GAME_PIECE) {
        reqs.addAll(Arrays.asList(GamePieceSettings.getIntakeRequirements()));
        break;
      }
    }

    return Commands.defer(
        () -> {
          List<Watch> watches = new ArrayList<>();
          for (ObstacleRegion r : path.getObstacleRegions()) {
            watches.add(new Watch(r));
          }
          return segment(
              path.getAllPathPoints(), path.getGlobalConstraints(), path.getGoalEndState(),
              watches, drive, reqs);
        },
        reqs);
  }

  /** Follow a stretch of path, stopping early if a region needs action. */
  private static Command segment(
      List<PathPoint> points,
      PathConstraints constraints,
      GoalEndState goal,
      List<Watch> watches,
      Subsystem[] drive,
      Set<Subsystem> reqs) {
    PathPlannerPath stretch = PathPlannerPath.fromPathPoints(points, constraints, goal);
    if (watches.isEmpty()) {
      return AutoBuilder.followPath(stretch);
    }

    Decision[] decision = {Decision.NONE};
    Watch[] hit = {null};

    Command follow =
        AutoBuilder.followPath(stretch)
            .until(
                () -> {
                  Decision d = checkAll(points, watches, decision, hit);
                  return d == Decision.STOP || d == Decision.DIVERT || d == Decision.PICKUP;
                });

    Command afterFollow =
        Commands.defer(
            () -> {
              if (decision[0] == Decision.DIVERT) {
                return diverge(hit[0].region);
              }
              if (decision[0] == Decision.PICKUP) {
                return pickup(points, constraints, goal, hit[0], drive, reqs);
              }
              return waitThenDecide(points, constraints, goal, watches, hit[0], drive, reqs);
            },
            reqs);

    return follow.andThen(Commands.either(afterFollow, Commands.none(), () -> hit[0] != null));
  }

  /** Stop and wait for the timeout, then resume or divert. */
  private static Command waitThenDecide(
      List<PathPoint> points,
      PathConstraints constraints,
      GoalEndState goal,
      List<Watch> watches,
      Watch watch,
      Subsystem[] drive,
      Set<Subsystem> reqs) {
    Decision[] result = {Decision.STOP};

    Command hold =
        Commands.run(ObstacleSettings.getStopDrive(), drive)
            .until(
                () -> {
                  // The condition can be checked again after it first returns true. Keep the
                  // first decision, or a second check would see the finished monitor and resume.
                  if (result[0] != Decision.STOP) return true;
                  Snapshot s = snapshot(points, watch.region);
                  result[0] =
                      watch.monitor.update(
                          s.inRegion, s.pastEnd, s.blocked, Timer.getFPGATimestamp());
                  return result[0] != Decision.STOP;
                });

    return hold.andThen(
        Commands.defer(
            () -> {
              if (result[0] == Decision.DIVERT) {
                return diverge(watch.region);
              }
              // Resume: follow the rest of the path from the robot's closest point
              int idx = nearestIndex(points);
              if (idx < 0) return Commands.none();
              List<PathPoint> rest = new ArrayList<>(points.subList(idx, points.size()));
              List<Watch> later = new ArrayList<>();
              for (Watch w : watches) {
                if (w != watch
                    && !w.isDone()
                    && w.region.getEndPos() > points.get(idx).waypointRelativePos) {
                  later.add(w);
                }
              }
              return segment(rest, constraints, goal, later, drive, reqs);
            },
            reqs));
  }

  /** Drive to the game piece with the intake running, then rejoin the main path or divert. */
  private static Command pickup(
      List<PathPoint> points,
      PathConstraints constraints,
      GoalEndState goal,
      Watch watch,
      Subsystem[] drive,
      Set<Subsystem> reqs) {
    Translation2d piece = watch.pieceTarget;
    Pose2d robot = ObstacleSettings.getPoseSupplier().get();
    // Point the intake at the piece
    Rotation2d heading =
        piece
            .minus(robot.getTranslation())
            .getAngle()
            .plus(Rotation2d.fromDegrees(GamePieceSettings.getIntakeFacingDeg()));

    // The piece position is a real field position, so it is not flipped
    Command arrive =
        AutoBuilder.pathfindToPose(new Pose2d(piece, heading), constraints)
            .andThen(
                Commands.run(ObstacleSettings.getStopDrive(), drive)
                    .withTimeout(GamePieceSettings.getIntakeAfterArriveSec()));

    BooleanSupplier hasPiece = GamePieceSettings.getHasPiece();
    Command grab = hasPiece == null ? arrive : arrive.until(hasPiece);
    grab =
        grab.deadlineFor(GamePieceSettings.makeIntakeCommand())
            .withTimeout(GamePieceSettings.getMaxPickupSec());

    Command after =
        Commands.defer(
            () -> {
              boolean divert =
                  ObstacleSettings.effectiveAction(watch.region.getAction()) == RegionAction.DIVERT
                      && divertPathFor(watch.region) != null;
              return divert
                  ? diverge(watch.region)
                  : rejoin(points, constraints, goal, watch.region);
            },
            reqs);

    return grab.andThen(after);
  }

  /** Go back to the main path where the region ends and finish it. */
  private static Command rejoin(
      List<PathPoint> points, PathConstraints constraints, GoalEndState goal, ObstacleRegion region) {
    List<PathPoint> rest = new ArrayList<>();
    for (PathPoint p : points) {
      if (p.waypointRelativePos >= region.getEndPos()) rest.add(p);
    }
    if (rest.size() < 2) {
      Translation2d end = points.get(points.size() - 1).position;
      return AutoBuilder.pathfindToPoseFlipped(new Pose2d(end, goal.rotation()), constraints);
    }
    PathPlannerPath restPath = PathPlannerPath.fromPathPoints(rest, constraints, goal);
    return AutoBuilder.pathfindThenFollowPath(restPath, constraints);
  }

  /** Check every region that is not done. Returns the first decision that needs action. */
  private static Decision checkAll(
      List<PathPoint> points, List<Watch> watches, Decision[] decision, Watch[] hit) {
    // Keep the first decision if this is checked again after it fired
    if (hit[0] != null) return decision[0];
    for (Watch w : watches) {
      if (w.isDone()) continue;
      Decision d;
      if (w.isPiece()) {
        if (!GamePieceSettings.isActive()) continue;
        d = checkPiece(points, w);
      } else {
        if (ObstacleSettings.getSource() == null) continue;
        Snapshot s = snapshot(points, w.region);
        d = w.monitor.update(s.inRegion, s.pastEnd, s.blocked, Timer.getFPGATimestamp());
      }
      if (d == Decision.STOP || d == Decision.DIVERT || d == Decision.PICKUP) {
        decision[0] = d;
        hit[0] = w;
        return d;
      }
    }
    return Decision.NONE;
  }

  /** PICKUP if the robot is in the region and a fresh game piece is close enough. */
  private static Decision checkPiece(List<PathPoint> points, Watch watch) {
    int idx = nearestIndex(points);
    if (idx < 0) return Decision.NONE;
    double pos = points.get(idx).waypointRelativePos;
    if (pos > watch.region.getEndPos()) {
      watch.pieceDone = true; // Passed the region without seeing a piece
      return Decision.NONE;
    }
    if (pos < watch.region.getStartPos()) return Decision.NONE;

    // Already holding a piece (beam break or other sensor): don't go get another one
    BooleanSupplier hasPiece = GamePieceSettings.getHasPiece();
    if (hasPiece != null && hasPiece.getAsBoolean()) return Decision.NONE;

    Pose2d robot = ObstacleSettings.getPoseSupplier().get();
    double now = Timer.getFPGATimestamp();
    Translation2d best = null;
    double bestDist = watch.region.getPieceMaxDistanceM();
    for (GamePiece g : GamePieceSettings.getSource().getGamePieces()) {
      if (now - g.timestampSec() > ObstacleSettings.getStaleAfterSec()) continue;
      Translation2d at = new Translation2d(g.position().x(), g.position().y());
      double dist = at.getDistance(robot.getTranslation());
      if (dist <= bestDist) {
        bestDist = dist;
        best = at;
      }
    }
    if (best == null) return Decision.NONE;
    watch.pieceTarget = best;
    watch.pieceDone = true;
    return Decision.PICKUP;
  }

  /** The diverge path to use: the dashboard pick wins over the region's own path. */
  private static PathPlannerPath divertPathFor(ObstacleRegion region) {
    PathPlannerPath picked = ObstacleSettings.getDivertOverride();
    return picked != null ? picked : region.getDivertPath();
  }

  /** The diverge path, followed from wherever the robot is. Nothing if there is none. */
  private static Command diverge(ObstacleRegion region) {
    PathPlannerPath divert = divertPathFor(region);
    if (divert == null) return Commands.none();
    return AutoBuilder.pathfindThenFollowPath(divert, divert.getGlobalConstraints());
  }

  /** What the region looks like right now. */
  private static final class Snapshot {
    boolean inRegion;
    boolean pastEnd;
    boolean blocked;
  }

  private static Snapshot snapshot(List<PathPoint> points, ObstacleRegion region) {
    Snapshot s = new Snapshot();
    int idx = nearestIndex(points);
    if (idx < 0) return s;

    double pos = points.get(idx).waypointRelativePos;
    s.inRegion = pos >= region.getStartPos() && pos <= region.getEndPos();
    s.pastEnd = pos > region.getEndPos();

    List<Point> regionPoints = new ArrayList<>();
    for (PathPoint p : points) {
      if (p.waypointRelativePos >= region.getStartPos()
          && p.waypointRelativePos <= region.getEndPos()) {
        Translation2d at = fieldPosition(p);
        regionPoints.add(new Point(at.getX(), at.getY()));
      }
    }

    ObstacleSource source = ObstacleSettings.getSource();
    s.blocked =
        source != null
            && RegionGeometry.blocked(
                source.getObstacles(),
                regionPoints,
                region.getTeam(),
                currentAlliance(),
                Timer.getFPGATimestamp(),
                ObstacleSettings.getStaleAfterSec(),
                BLOCK_RADIUS_METERS);
    return s;
  }

  /** Index of the path point closest to the robot right now. -1 if there are none. */
  private static int nearestIndex(List<PathPoint> points) {
    Pose2d pose = ObstacleSettings.getPoseSupplier().get();
    double[] xs = new double[points.size()];
    double[] ys = new double[points.size()];
    for (int i = 0; i < points.size(); i++) {
      Translation2d at = fieldPosition(points.get(i));
      xs[i] = at.getX();
      ys[i] = at.getY();
    }
    return RegionGeometry.nearestIndex(xs, ys, pose.getX(), pose.getY());
  }

  /**
   * Where a path point is on the real field. Paths are saved on the blue side, and the path
   * follower flips them on red, so flip here too to match the robot pose and the obstacles.
   */
  private static Translation2d fieldPosition(PathPoint p) {
    return AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPosition(p.position) : p.position;
  }

  private static AllianceColor currentAlliance() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) return AllianceColor.UNKNOWN;
    return alliance.get() == DriverStation.Alliance.Red ? AllianceColor.RED : AllianceColor.BLUE;
  }
}
