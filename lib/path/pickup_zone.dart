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
    // On a curvy region the box's corners can be farther out than a corner may be dragged, so
    // pull them in to the same limit
    return [
      for (final c in [
        corner(minA, minB),
        corner(maxA, minB),
        corner(maxA, maxB),
        corner(minA, maxB),
      ])
        clampCorner(path, region, c),
    ];
  }

  /// True if the point is inside the zone
  static bool contains(List<Translation2d> zone, Translation2d p) {
    bool inside = false;
    for (int i = 0, j = zone.length - 1; i < zone.length; j = i++) {
      final a = zone[i], b = zone[j];
      if ((a.y > p.y) != (b.y > p.y) &&
          p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) {
        inside = !inside;
      }
    }
    return inside;
  }

  /// True if no two edges of the zone cross. A crossed zone has no clear inside.
  static bool isSimple(List<Translation2d> zone) {
    final int n = zone.length;
    if (n < 4) return true;
    for (int i = 0; i < n; i++) {
      for (int j = i + 1; j < n; j++) {
        // Edges that share a corner always touch
        if (j == i + 1 || (i == 0 && j == n - 1)) continue;
        if (_segmentsCross(
            zone[i], zone[(i + 1) % n], zone[j], zone[(j + 1) % n])) {
          return false;
        }
      }
    }
    return true;
  }

  static bool _segmentsCross(
      Translation2d a, Translation2d b, Translation2d c, Translation2d d) {
    num cross(Translation2d o, Translation2d p, Translation2d q) =>
        (p.x - o.x) * (q.y - o.y) - (p.y - o.y) * (q.x - o.x);
    final num d1 = cross(c, d, a);
    final num d2 = cross(c, d, b);
    final num d3 = cross(a, b, c);
    final num d4 = cross(a, b, d);
    return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
        ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
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
