package com.mercury1089.reactivepathing;

import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

/**
 * Helpers for working with paths and the field.
 *
 * <p>The biggest one is {@link #restOfPath}, used when the robot goes back to a path after waiting
 * or picking up a game piece. Each part of a path between two waypoints is a Bezier curve, shaped by
 * four points (two anchors and two control points). De Casteljau's method splits a Bezier curve at
 * any point into two smaller curves with exactly the same shape. We keep the second one.
 */
class PathTools {

  /**
   * The part of the path from a position to the end.
   *
   * @param path the whole path
   * @param cutPos where to start, as a waypoint-relative position
   * @return the rest of the path, or null if there is almost nothing left
   */
  static PathPlannerPath restOfPath(PathPlannerPath path, double cutPos) {
    List<Waypoint> waypoints = path.getWaypoints();
    int segments = waypoints.size() - 1;
    if (cutPos <= 0.0) {
      return path;
    }
    if (cutPos >= segments - 0.01) {
      return null;
    }

    int k = (int) Math.floor(cutPos); // which part of the path the cut is in
    double t = cutPos - k; // how far along that part, from 0 to 1

    // Split the curve between waypoint k and waypoint k + 1
    Translation2d p0 = waypoints.get(k).anchor();
    Translation2d p1 = waypoints.get(k).nextControl();
    Translation2d p2 = waypoints.get(k + 1).prevControl();
    Translation2d p3 = waypoints.get(k + 1).anchor();

    Translation2d p01 = lerp(p0, p1, t);
    Translation2d p12 = lerp(p1, p2, t);
    Translation2d p23 = lerp(p2, p3, t);
    Translation2d p012 = lerp(p01, p12, t);
    Translation2d p123 = lerp(p12, p23, t);
    Translation2d cutPoint = lerp(p012, p123, t);

    // The new waypoints: the cut point, then the rest of the old ones
    List<Waypoint> newWaypoints = new ArrayList<>();
    newWaypoints.add(new Waypoint(null, cutPoint, p123));
    newWaypoints.add(new Waypoint(p23, p3, waypoints.get(k + 1).nextControl()));
    for (int i = k + 2; i < waypoints.size(); i++) {
      newWaypoints.add(waypoints.get(i));
    }

    // Keep what comes after the cut, at its new position
    List<RotationTarget> rotations = new ArrayList<>();
    for (RotationTarget target : path.getRotationTargets()) {
      if (target.position() >= cutPos) {
        rotations.add(new RotationTarget(newPosition(target.position(), cutPos), target.rotation()));
      }
    }

    List<ConstraintsZone> constraintZones = new ArrayList<>();
    for (ConstraintsZone zone : path.getConstraintZones()) {
      if (zone.maxPosition() > cutPos) {
        double start = Math.max(zone.minPosition(), cutPos);
        constraintZones.add(
            new ConstraintsZone(
                newPosition(start, cutPos),
                newPosition(zone.maxPosition(), cutPos),
                zone.constraints()));
      }
    }

    List<PointTowardsZone> pointZones = new ArrayList<>();
    for (PointTowardsZone zone : path.getPointTowardsZones()) {
      if (zone.maxPosition() > cutPos) {
        double start = Math.max(zone.minPosition(), cutPos);
        pointZones.add(
            new PointTowardsZone(
                zone.name(),
                zone.targetPosition(),
                zone.rotationOffset(),
                newPosition(start, cutPos),
                newPosition(zone.maxPosition(), cutPos)));
      }
    }

    List<EventMarker> markers = new ArrayList<>();
    for (EventMarker marker : path.getEventMarkers()) {
      boolean isZone = marker.endPosition() >= 0.0;
      if (!isZone && marker.position() >= cutPos) {
        markers.add(
            new EventMarker(
                marker.triggerName(),
                newPosition(marker.position(), cutPos),
                -1.0,
                marker.command()));
      }
      if (isZone && marker.endPosition() > cutPos) {
        double start = Math.max(marker.position(), cutPos);
        markers.add(
            new EventMarker(
                marker.triggerName(),
                newPosition(start, cutPos),
                newPosition(marker.endPosition(), cutPos),
                marker.command()));
      }
    }

    PathPlannerPath rest =
        new PathPlannerPath(
            newWaypoints,
            rotations,
            pointZones,
            constraintZones,
            markers,
            path.getGlobalConstraints(),
            null, // the robot is already moving, so there is no ideal starting state
            path.getGoalEndState(),
            path.isReversed());
    rest.name = path.name;
    return rest;
  }

  /**
   * Where an old position lands on the rest of the path. Inside the cut part, positions are
   * stretched to fill 0 to 1. After it, they just move down by the parts that were cut off.
   */
  static double newPosition(double oldPos, double cutPos) {
    int k = (int) Math.floor(cutPos);
    double t = cutPos - k;
    if (oldPos < k + 1) {
      return (oldPos - cutPos) / (1.0 - t);
    }
    return oldPos - k;
  }

  private static Translation2d lerp(Translation2d a, Translation2d b, double t) {
    return a.interpolate(b, t);
  }

  /** Mirror a position to the other side of the field, the way a mirrored auto does. */
  static Translation2d mirror(Translation2d position) {
    return new Translation2d(position.getX(), FlippingUtil.fieldSizeY - position.getY());
  }

  /**
   * The way a holonomic robot faces at a point on the path: the starting rotation, then each
   * rotation target, then the end rotation, blended in between.
   *
   * @return the rotation, or null if the path has none
   */
  static Rotation2d rotationAt(PathPlannerPath path, double pos) {
    List<Double> positions = new ArrayList<>();
    List<Rotation2d> rotations = new ArrayList<>();
    if (path.getIdealStartingState() != null) {
      positions.add(0.0);
      rotations.add(path.getIdealStartingState().rotation());
    }
    List<RotationTarget> targets = new ArrayList<>(path.getRotationTargets());
    targets.sort((a, b) -> Double.compare(a.position(), b.position()));
    for (RotationTarget target : targets) {
      positions.add(target.position());
      rotations.add(target.rotation());
    }
    List<PathPoint> points = path.getAllPathPoints();
    if (path.getGoalEndState() != null && !points.isEmpty()) {
      positions.add(points.get(points.size() - 1).waypointRelativePos);
      rotations.add(path.getGoalEndState().rotation());
    }

    if (positions.isEmpty()) {
      return null;
    }
    if (pos <= positions.get(0)) {
      return rotations.get(0);
    }
    for (int i = 1; i < positions.size(); i++) {
      if (pos <= positions.get(i)) {
        double span = positions.get(i) - positions.get(i - 1);
        double t = 1.0;
        if (span > 1e-9) {
          t = (pos - positions.get(i - 1)) / span;
        }
        return rotations.get(i - 1).interpolate(rotations.get(i), t);
      }
    }
    return rotations.get(rotations.size() - 1);
  }

  /** @return true if a point is inside a polygon (the pickup zone) */
  static boolean isInside(List<Translation2d> polygon, Translation2d point) {
    boolean inside = false;
    int j = polygon.size() - 1;
    for (int i = 0; i < polygon.size(); i++) {
      Translation2d a = polygon.get(i);
      Translation2d b = polygon.get(j);
      boolean crosses = (a.getY() > point.getY()) != (b.getY() > point.getY());
      if (crosses) {
        double slope = (b.getX() - a.getX()) / (b.getY() - a.getY());
        double xAtY = a.getX() + slope * (point.getY() - a.getY());
        if (point.getX() < xAtY) {
          inside = !inside;
        }
      }
      j = i;
    }
    return inside;
  }
}
