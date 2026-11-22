import 'dart:math';

import 'package:pathplanner/path/obstacle_region.dart';
import 'package:pathplanner/path/pathplanner_path.dart';
import 'package:pathplanner/util/wpimath/geometry.dart';

/// Pickup zone shape rules for game piece regions: the default zone that surrounds a region, and
/// how far a zone corner may be dragged away from it.
class PickupZone {
  /// Space added past each end of the region, in meters
  static const double endPadding = 0.4;

  /// Space added on each side of the region, in meters
  static const double sidePadding = 0.6;

  /// Smallest and largest distance a corner may be from the region, in meters
  static const double minReach = 1.0;
  static const double maxReach = 4.0;

  /// How far a corner may be from the region, as a share of the region's length
  static const double reachPerMeter = 0.75;

  static const double _step = 0.05;

  /// Points along the part of the path the region covers
  static List<Translation2d> regionPoints(
      PathPlannerPath path, ObstacleRegion region) {
    final num start = region.startPos;
    final num end = max(region.startPos, region.endPos);
    final List<Translation2d> points = [path.samplePath(start)];
    for (num t = start + _step; t < end; t += _step) {
      points.add(path.samplePath(t));
    }
    if (end > start) points.add(path.samplePath(end));
    return points;
  }

  /// Length of the part of the path the region covers, in meters
  static double regionLength(PathPlannerPath path, ObstacleRegion region) {
    final points = regionPoints(path, region);
    double length = 0;
    for (int i = 1; i < points.length; i++) {
      length += points[i].getDistance(points[i - 1]).toDouble();
    }
    return length;
  }

  /// How far a zone corner may be from the closest point of the region
  static double reach(PathPlannerPath path, ObstacleRegion region) {
    return (regionLength(path, region) * reachPerMeter)
        .clamp(minReach, maxReach)
        .toDouble();
  }

  /// A rectangle lined up with the region that surrounds the whole region
  static List<Translation2d> defaultZone(
      PathPlannerPath path, ObstacleRegion region) {
    final points = regionPoints(path, region);
    Translation2d dir = points.last - points.first;
    if (dir.norm < 0.05) {
      // A region with no length: use the path direction there, or an upright square
      final num t = region.startPos;
      final Translation2d ahead =
          path.samplePath(t + 0.05) - path.samplePath(t);
      dir = ahead.norm > 1e-6 ? ahead : const Translation2d(1, 0);
    }
    final Translation2d u = dir / dir.norm;
    final Translation2d n = Translation2d(-u.y, u.x);

    double minA = double.infinity, maxA = -double.infinity;
    double minB = double.infinity, maxB = -double.infinity;
    for (final p in points) {
      final double a = (p.x * u.x + p.y * u.y).toDouble();
      final double b = (p.x * n.x + p.y * n.y).toDouble();
      minA = min(minA, a);
      maxA = max(maxA, a);
      minB = min(minB, b);
      maxB = max(maxB, b);
    }
    minA -= endPadding;
    maxA += endPadding;
    minB -= sidePadding;
    maxB += sidePadding;

    Translation2d corner(double a, double b) => u * a + n * b;
    return [
      corner(minA, minB),
      corner(maxA, minB),
      corner(maxA, maxB),
      corner(minA, maxB),
    ];
  }

  /// Keep a dragged corner within reach of the region
  static Translation2d clampCorner(
      PathPlannerPath path, ObstacleRegion region, Translation2d corner) {
    final points = regionPoints(path, region);
    Translation2d closest = points.first;
    num best = double.infinity;
    for (int i = 0; i < points.length; i++) {
      final Translation2d q = i + 1 < points.length
          ? _closestOnSegment(points[i], points[i + 1], corner)
          : points[i];
      final num d = q.getDistance(corner);
      if (d < best) {
        best = d;
        closest = q;
      }
    }
    final double limit = reach(path, region);
    if (best <= limit) return corner;
    return closest + (corner - closest) * (limit / best);
  }

  static Translation2d _closestOnSegment(
      Translation2d a, Translation2d b, Translation2d p) {
    final Translation2d ab = b - a;
    final num len2 = ab.x * ab.x + ab.y * ab.y;
    if (len2 < 1e-12) return a;
    final num t =
        (((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / len2).clamp(0, 1);
    return a + ab * t;
  }
}
