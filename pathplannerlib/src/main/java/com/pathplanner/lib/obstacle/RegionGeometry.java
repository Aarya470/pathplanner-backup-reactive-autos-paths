package com.pathplanner.lib.obstacle;

import java.util.List;

/** Geometry helpers for obstacle regions. Plain Java, so they can be tested on their own. */
public final class RegionGeometry {
  private RegionGeometry() {}

  /**
   * Find the point closest to the robot.
   *
   * @param xs point x values
   * @param ys point y values
   * @param robotX robot x, meters
   * @param robotY robot y, meters
   * @return the index of the closest point, or -1 if there are no points
   */
  public static int nearestIndex(double[] xs, double[] ys, double robotX, double robotY) {
    int best = -1;
    double bestDist = Double.POSITIVE_INFINITY;
    for (int i = 0; i < xs.length; i++) {
      double dx = xs[i] - robotX;
      double dy = ys[i] - robotY;
      double dist = dx * dx + dy * dy;
      if (dist < bestDist) {
        bestDist = dist;
        best = i;
      }
    }
    return best;
  }

  /**
   * True if a fresh obstacle that counts for this region is within [radiusM] of any point of the
   * region.
   *
   * @param obstacles the latest obstacles
   * @param regionPoints the points of the path inside the region
   * @param team which robots count
   * @param ourAlliance our alliance, from the FMS
   * @param nowSec the current time, seconds
   * @param staleAfterSec obstacles older than this are ignored
   * @param radiusM how close an obstacle must be to the path to block it
   * @return true if the region is blocked
   */
  public static boolean blocked(
      List<Obstacle> obstacles,
      List<Point> regionPoints,
      ObstacleTeam team,
      AllianceColor ourAlliance,
      double nowSec,
      double staleAfterSec,
      double radiusM) {
    for (Obstacle o : obstacles) {
      if (nowSec - o.timestampSec() > staleAfterSec) continue;
      if (!countsFor(team, o.color(), ourAlliance)) continue;
      for (Point p : regionPoints) {
        double dx = o.position().x() - p.x();
        double dy = o.position().y() - p.y();
        if (Math.hypot(dx, dy) <= radiusM) return true;
      }
    }
    return false;
  }

  /** Whether an obstacle of this color counts for the given team setting. */
  static boolean countsFor(ObstacleTeam team, AllianceColor color, AllianceColor ourAlliance) {
    if (team == ObstacleTeam.ANY) return true;
    if (ourAlliance == AllianceColor.UNKNOWN || color == AllianceColor.UNKNOWN) return false;
    if (team == ObstacleTeam.TEAMMATE) return color == ourAlliance;
    return color != ourAlliance;
  }
}
