import 'dart:math';

import 'package:pathplanner/path/obstacle_region.dart';
import 'package:pathplanner/path/pathplanner_path.dart';
import 'package:pathplanner/path/waypoint.dart';
import 'package:pathplanner/util/wpimath/geometry.dart';

/// The route a robot drives when it takes a region's pivot: the main path up to the end of the
/// region, then the pivot path. Zones, rotation targets and event markers from both come along.
class PivotRoute {
  /// The main path up to the exact end of the region, then the pivot path. The original curve is
  /// split at the region's end, so the pivot leaves the original path exactly there.
  static PathPlannerPath build(PathPlannerPath main, ObstacleRegion region) {
    final PathPlannerPath divert = region.divertPath!;
    final int n = main.waypoints.length;
    final int i = min(region.endPos.floor(), n - 2).clamp(0, n - 2);
    final double t = (region.endPos - i).clamp(0.0, 1.0).toDouble();

    final Waypoint a = main.waypoints[i];
    final Waypoint b = main.waypoints[i + 1];
    final Translation2d p0 = a.anchor;
    final Translation2d p1 = a.nextControl ?? a.anchor;
    final Translation2d p2 = b.prevControl ?? b.anchor;
    final Translation2d p3 = b.anchor;

    // De Casteljau split of the segment at t. The left half is p0, q1, r1, split.
    final Translation2d q1 = _lerp(p0, p1, t);
    final Translation2d q2 = _lerp(p1, p2, t);
    final Translation2d q3 = _lerp(p2, p3, t);
    final Translation2d r1 = _lerp(q1, q2, t);
    final Translation2d r2 = _lerp(q2, q3, t);
    final Translation2d split = _lerp(r1, r2, t);

    final List<Waypoint> route =
        PathPlannerPath.cloneWaypoints(main.waypoints.sublist(0, i + 1));
    route.last.nextControl = q1;
    final List<Waypoint> pivot =
        PathPlannerPath.cloneWaypoints(divert.waypoints);
    if (pivot.isNotEmpty) {
      pivot.first.anchor = split;
      pivot.first.prevControl = r1;
    }

    final PathPlannerPath result = PathPlannerPath.defaultPath(
      pathDir: main.pathDir,
      fs: main.fs,
      name: '${main.name} pivot route',
    );
    result.waypoints = [...route, ...pivot];
    for (final w in result.waypoints) {
      w.prevControl ??= w.anchor;
      w.nextControl ??= w.anchor;
    }
    result.globalConstraints = main.globalConstraints.clone();
    result.useDefaultConstraints = main.useDefaultConstraints;
    result.reversed = main.reversed;
    result.idealStartingState = main.idealStartingState;
    result.goalEndState = divert.goalEndState.clone();

    // Main path positions before the split keep their place. The split segment is squeezed from
    // [i, i + t] onto [i, i + 1]. Pivot path positions move up by i + 1.
    final num splitPos = i + t;
    num mainToRoute(num pos) {
      if (pos <= i || t <= 0) return pos;
      return i + (pos - i) / t;
    }

    num pivotToRoute(num pos) => pos + i + 1;

    result.constraintZones = [
      for (final z in main.constraintZones)
        if (z.minWaypointRelativePos < splitPos)
          z.clone()
            ..minWaypointRelativePos = mainToRoute(z.minWaypointRelativePos)
            ..maxWaypointRelativePos =
                mainToRoute(min(z.maxWaypointRelativePos, splitPos)),
      for (final z in divert.constraintZones)
        z.clone()
          ..minWaypointRelativePos = pivotToRoute(z.minWaypointRelativePos)
          ..maxWaypointRelativePos = pivotToRoute(z.maxWaypointRelativePos),
    ];
    result.pointTowardsZones = [
      for (final z in main.pointTowardsZones)
        if (z.minWaypointRelativePos < splitPos)
          z.clone()
            ..minWaypointRelativePos = mainToRoute(z.minWaypointRelativePos)
            ..maxWaypointRelativePos =
                mainToRoute(min(z.maxWaypointRelativePos, splitPos)),
      for (final z in divert.pointTowardsZones)
        z.clone()
          ..minWaypointRelativePos = pivotToRoute(z.minWaypointRelativePos)
          ..maxWaypointRelativePos = pivotToRoute(z.maxWaypointRelativePos),
    ];
    result.rotationTargets = [
      for (final r in main.rotationTargets)
        if (r.waypointRelativePos < splitPos)
          r.clone()..waypointRelativePos = mainToRoute(r.waypointRelativePos),
      for (final r in divert.rotationTargets)
        r.clone()..waypointRelativePos = pivotToRoute(r.waypointRelativePos),
    ];
    result.eventMarkers = [
      for (final m in main.eventMarkers)
        if (m.waypointRelativePos < splitPos)
          m.clone()
            ..waypointRelativePos = mainToRoute(m.waypointRelativePos)
            ..endWaypointRelativePos = m.endWaypointRelativePos == null
                ? null
                : mainToRoute(min(m.endWaypointRelativePos!, splitPos)),
      for (final m in divert.eventMarkers)
        m.clone()
          ..waypointRelativePos = pivotToRoute(m.waypointRelativePos)
          ..endWaypointRelativePos = m.endWaypointRelativePos == null
              ? null
              : pivotToRoute(m.endWaypointRelativePos!),
    ];

    result.generatePathPoints();
    return result;
  }

  static Translation2d _lerp(Translation2d a, Translation2d b, double t) {
    return Translation2d(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
  }
}
