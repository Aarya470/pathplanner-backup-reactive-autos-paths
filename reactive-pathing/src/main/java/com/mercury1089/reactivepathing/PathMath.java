package com.mercury1089.reactivepathing;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

/** Small helpers for working with paths and the field. */
public class PathMath {
  private PathMath() {}

  /**
   * Flip a position to the red side of the field. Paths are drawn on the blue side, and
   * PathPlanner flips them on red, so anything drawn in the app is flipped the same way.
   */
  public static Translation2d flipToRed(Translation2d bluePosition) {
    return FlippingUtil.flipFieldPosition(bluePosition);
  }

  /** Mirror a position to the other side of the field, the way a mirrored auto does. */
  public static Translation2d mirror(Translation2d position) {
    return new Translation2d(position.getX(), FlippingUtil.fieldSizeY - position.getY());
  }

  /**
   * Find the path point closest to a position, only looking at points from fromPos to toPos along
   * the path. Looking only a little way ahead stops a path that comes back near itself (out and
   * back) from jumping to the wrong part of the path.
   *
   * @param points where each path point is on the field
   * @param pointPos where each path point is along the path
   * @param to the position to be close to, such as the robot
   * @param fromPos only look at points at or after this position along the path
   * @param toPos only look at points at or before this position along the path
   * @return the index of the closest point, or -1 if there are none in that part of the path
   */
  public static int closestPointIndex(
      List<Translation2d> points,
      List<Double> pointPos,
      Translation2d to,
      double fromPos,
      double toPos) {
    int best = -1;
    double bestDistance = Double.POSITIVE_INFINITY;
    for (int i = 0; i < points.size(); i++) {
      if (pointPos.get(i) < fromPos || pointPos.get(i) > toPos) {
        continue;
      }
      double distance = points.get(i).getDistance(to);
      if (distance < bestDistance) {
        bestDistance = distance;
        best = i;
      }
    }
    return best;
  }

  /**
   * The way a holonomic robot faces at a point on the path: the starting rotation, then each
   * rotation target, then the end rotation, blended in between.
   *
   * @return the rotation, or null if the path has none
   */
  public static Rotation2d rotationAt(PathPlannerPath path, double pos) {
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
  public static boolean isInside(List<Translation2d> polygon, Translation2d point) {
    boolean inside = false;
    int j = polygon.size() - 1;
    for (int i = 0; i < polygon.size(); i++) {
      Translation2d a = polygon.get(i);
      Translation2d b = polygon.get(j);
      boolean crosses = (a.getY() > point.getY()) != (b.getY() > point.getY());
      if (crosses) {
        double xAtY = (b.getX() - a.getX()) * (point.getY() - a.getY()) / (b.getY() - a.getY()) + a.getX();
        if (point.getX() < xAtY) {
          inside = !inside;
        }
      }
      j = i;
    }
    return inside;
  }
}
