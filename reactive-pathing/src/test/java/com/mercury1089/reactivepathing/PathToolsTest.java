package com.mercury1089.reactivepathing;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;

import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.junit.jupiter.api.Test;

public class PathToolsTest extends WpilibTest {
  /** A curvy path with 3 waypoints and one of everything */
  static PathPlannerPath curvyPath() {
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(1, 1, Rotation2d.fromDegrees(0)),
            new Pose2d(4, 3, Rotation2d.fromDegrees(45)),
            new Pose2d(7, 2, Rotation2d.fromDegrees(-30)));
    PathConstraints slow = new PathConstraints(1, 1, 3, 3);
    return new PathPlannerPath(
        waypoints,
        List.of(new RotationTarget(0.3, Rotation2d.fromDegrees(10)), new RotationTarget(1.5, Rotation2d.fromDegrees(90))),
        List.of(new PointTowardsZone("look", new Translation2d(8, 4), Rotation2d.kZero, 0.2, 1.4)),
        List.of(new ConstraintsZone(0.6, 1.2, slow)),
        List.of(
            new EventMarker("early", 0.2, null),
            new EventMarker("late", 1.7, null),
            new EventMarker("zone", 0.5, 1.5, null)),
        new PathConstraints(3, 3, 6, 6),
        null,
        new GoalEndState(0, Rotation2d.fromDegrees(-30)),
        false);
  }

  @Test
  public void cuttingAtZeroGivesTheSamePath() {
    PathPlannerPath path = curvyPath();
    assertSame(path, PathTools.restOfPath(path, 0.0));
  }

  @Test
  public void cuttingAtTheEndGivesNothing() {
    assertNull(PathTools.restOfPath(curvyPath(), 2.0));
  }

  @Test
  public void theRestFollowsTheSameCurve() {
    PathPlannerPath path = curvyPath();
    double cut = 0.6;
    PathPlannerPath rest = PathTools.restOfPath(path, cut);

    // Starts at the original path's point at the cut, and ends at the same place
    Translation2d expectedStart = pointAt(path, cut);
    assertEquals(expectedStart.getX(), rest.getWaypoints().get(0).anchor().getX(), 1e-9);
    assertEquals(expectedStart.getY(), rest.getWaypoints().get(0).anchor().getY(), 1e-9);
    assertEquals(path.getWaypoints().get(2).anchor(), rest.getWaypoints().get(2).anchor());

    // Every point of the rest lies on the original curve
    for (PathPoint point : rest.getAllPathPoints()) {
      double distance = distanceToPath(path, point.position);
      assertEquals(0.0, distance, 0.02, "point " + point.position + " is off the original path");
    }
  }

  @Test
  public void markersTargetsAndZonesMoveWithTheCut() {
    // Cut at 0.6, inside the first part. The rest still has two parts: the cut point to waypoint
    // 1, then waypoint 1 to 2. So 0.6 to 1 stretches to 0 to 1, and positions after 1 stay put.
    PathPlannerPath rest = PathTools.restOfPath(curvyPath(), 0.6);

    // "early" (0.2) is before the cut and is dropped
    List<EventMarker> markers = rest.getEventMarkers();
    assertEquals(2, markers.size());
    EventMarker zone = markers.get(0);
    assertEquals("zone", zone.triggerName());
    assertEquals(0.0, zone.position(), 1e-9); // it was already active at the cut
    assertEquals(1.5, zone.endPosition(), 1e-9);
    assertEquals("late", markers.get(1).triggerName());
    assertEquals(1.7, markers.get(1).position(), 1e-9);

    // Rotation target at 0.3 is dropped, 1.5 stays
    assertEquals(1, rest.getRotationTargets().size());
    assertEquals(1.5, rest.getRotationTargets().get(0).position(), 1e-9);

    // The slow zone 0.6 to 1.2 starts right at the cut
    ConstraintsZone slow = rest.getConstraintZones().get(0);
    assertEquals(0.0, slow.minPosition(), 1e-9);
    assertEquals(1.2, slow.maxPosition(), 1e-9);

    PointTowardsZone look = rest.getPointTowardsZones().get(0);
    assertEquals(0.0, look.minPosition(), 1e-9);
    assertEquals(1.4, look.maxPosition(), 1e-9);
  }

  @Test
  public void cuttingInTheSecondPartShiftsPositionsDown() {
    // Cut at 1.25: the rest is one part, from the cut to the last waypoint
    PathPlannerPath rest = PathTools.restOfPath(curvyPath(), 1.25);
    assertEquals(2, rest.getWaypoints().size());
    // "late" at 1.7 becomes (1.7 - 1.25) / (1 - 0.25) = 0.6
    assertEquals(1, rest.getEventMarkers().stream().filter(m -> m.triggerName().equals("late")).count());
    for (EventMarker m : rest.getEventMarkers()) {
      if (m.triggerName().equals("late")) {
        assertEquals(0.6, m.position(), 1e-9);
      }
    }
    // Rotation target 1.5 becomes (1.5 - 1.25) / 0.75 = 1/3
    assertEquals(1.0 / 3.0, rest.getRotationTargets().get(0).position(), 1e-9);
  }

  @Test
  public void positionsInsideTheCutPartStretch() {
    assertEquals(0.5, PathTools.newPosition(0.8, 0.6), 1e-9); // (0.8 - 0.6) / 0.4
    assertEquals(0.0, PathTools.newPosition(0.6, 0.6), 1e-9);
    assertEquals(1.25, PathTools.newPosition(2.25, 1.0), 1e-9);
  }

  /** The original path's point at a position, worked out from its Bezier curves */
  static Translation2d pointAt(PathPlannerPath path, double pos) {
    int k = (int) Math.floor(pos);
    double t = pos - k;
    Waypoint a = path.getWaypoints().get(k);
    Waypoint b = path.getWaypoints().get(k + 1);
    double u = 1 - t;
    double x =
        u * u * u * a.anchor().getX()
            + 3 * u * u * t * a.nextControl().getX()
            + 3 * u * t * t * b.prevControl().getX()
            + t * t * t * b.anchor().getX();
    double y =
        u * u * u * a.anchor().getY()
            + 3 * u * u * t * a.nextControl().getY()
            + 3 * u * t * t * b.prevControl().getY()
            + t * t * t * b.anchor().getY();
    return new Translation2d(x, y);
  }

  static double distanceToPath(PathPlannerPath path, Translation2d point) {
    double best = Double.POSITIVE_INFINITY;
    int segments = path.getWaypoints().size() - 1;
    for (int i = 0; i <= segments * 2000; i++) {
      double pos = Math.min(segments - 1e-9, i / 2000.0);
      best = Math.min(best, pointAt(path, pos).getDistance(point));
    }
    return best;
  }
}
