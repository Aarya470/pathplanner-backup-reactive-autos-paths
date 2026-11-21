package com.pathplanner.lib.obstacle;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * The heading a holonomic robot has along a path: the starting rotation, then each rotation
 * target, then the end rotation, blended in between. Used to face the right way when the robot
 * rejoins the path after a pickup.
 */
public final class RotationProfile {
  private final double[] positions;
  private final Rotation2d[] rotations;

  /**
   * Make a profile from known rotations.
   *
   * @param positions waypoint-relative positions, in increasing order
   * @param rotations the rotation at each position
   */
  public RotationProfile(double[] positions, Rotation2d[] rotations) {
    this.positions = positions;
    this.rotations = rotations;
  }

  /**
   * The profile of a path.
   *
   * @param path the path
   * @return its rotation profile
   */
  public static RotationProfile of(PathPlannerPath path) {
    List<double[]> pos = new ArrayList<>();
    List<Rotation2d> rot = new ArrayList<>();
    if (path.getIdealStartingState() != null) {
      pos.add(new double[] {0.0});
      rot.add(path.getIdealStartingState().rotation());
    }
    List<RotationTarget> targets = new ArrayList<>(path.getRotationTargets());
    targets.sort(Comparator.comparingDouble(RotationTarget::position));
    for (RotationTarget t : targets) {
      pos.add(new double[] {t.position()});
      rot.add(t.rotation());
    }
    if (path.getGoalEndState() != null && !path.getAllPathPoints().isEmpty()) {
      double last =
          path.getAllPathPoints().get(path.getAllPathPoints().size() - 1).waypointRelativePos;
      pos.add(new double[] {last});
      rot.add(path.getGoalEndState().rotation());
    }
    double[] p = new double[pos.size()];
    for (int i = 0; i < p.length; i++) p[i] = pos.get(i)[0];
    return new RotationProfile(p, rot.toArray(new Rotation2d[0]));
  }

  /**
   * The heading at a point on the path.
   *
   * @param pos waypoint-relative position
   * @return the heading, or null if the path has no rotations at all
   */
  public Rotation2d at(double pos) {
    if (positions.length == 0) return null;
    if (pos <= positions[0]) return rotations[0];
    for (int i = 1; i < positions.length; i++) {
      if (pos <= positions[i]) {
        double span = positions[i] - positions[i - 1];
        double t = span <= 1e-9 ? 1.0 : (pos - positions[i - 1]) / span;
        return rotations[i - 1].interpolate(rotations[i], t);
      }
    }
    return rotations[rotations.length - 1];
  }
}
