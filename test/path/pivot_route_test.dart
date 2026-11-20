import 'package:file/memory.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:pathplanner/path/constraints_zone.dart';
import 'package:pathplanner/path/obstacle_region.dart';
import 'package:pathplanner/path/path_constraints.dart';
import 'package:pathplanner/path/pathplanner_path.dart';
import 'package:pathplanner/path/pivot_route.dart';
import 'package:pathplanner/path/rotation_target.dart';
import 'package:pathplanner/util/wpimath/geometry.dart';

void main() {
  final fs = MemoryFileSystem();

  PathPlannerPath mainPath() {
    final path =
        PathPlannerPath.defaultPath(pathDir: '/paths', fs: fs, name: 'main');
    path.addWaypoint(const Translation2d(6.0, 6.0));
    // Waypoints at positions 0, 1, 2
    path.constraintZones = [
      ConstraintsZone(
          minWaypointRelativePos: 0.2,
          maxWaypointRelativePos: 0.6,
          constraints: PathConstraints(maxVelocityMPS: 1.0)),
      ConstraintsZone(
          minWaypointRelativePos: 1.8,
          maxWaypointRelativePos: 2.0,
          constraints: PathConstraints(maxVelocityMPS: 0.5)),
    ];
    path.rotationTargets = [
      RotationTarget(0.5, Rotation2d.fromDegrees(90)),
      RotationTarget(1.9, Rotation2d.fromDegrees(45)),
    ];
    return path;
  }

  ObstacleRegion regionWithPivot() {
    final pivot =
        PathPlannerPath.defaultPath(pathDir: '/paths', fs: fs, name: 'pivot');
    pivot.isNested = true;
    pivot.constraintZones = [
      ConstraintsZone(
          minWaypointRelativePos: 0.0,
          maxWaypointRelativePos: 1.0,
          constraints: PathConstraints(maxVelocityMPS: 2.0)),
    ];
    pivot.rotationTargets = [RotationTarget(0.5, Rotation2d.fromDegrees(180))];
    pivot.goalEndState.rotation = Rotation2d.fromDegrees(-90);
    return ObstacleRegion(
        startPos: 0.8,
        endPos: 1.5,
        action: RegionAction.divert,
        divertPath: pivot);
  }

  test('main path zones before the break-off are kept, later ones dropped', () {
    final route = PivotRoute.build(mainPath(), regionWithPivot());

    final slow = route.constraintZones
        .where((z) => z.constraints.maxVelocityMPS == 1.0)
        .toList();
    expect(slow.length, 1);
    expect(slow.first.minWaypointRelativePos, closeTo(0.2, 1e-9));
    expect(slow.first.maxWaypointRelativePos, closeTo(0.6, 1e-9));

    expect(
        route.constraintZones.any((z) => z.constraints.maxVelocityMPS == 0.5),
        false);
  });

  test('pivot zones and rotations move to where the pivot starts', () {
    final route = PivotRoute.build(mainPath(), regionWithPivot());

    // The break-off is in segment 1, so the pivot starts at route waypoint 2
    final pivotZone = route.constraintZones
        .firstWhere((z) => z.constraints.maxVelocityMPS == 2.0);
    expect(pivotZone.minWaypointRelativePos, closeTo(2.0, 1e-9));
    expect(pivotZone.maxWaypointRelativePos, closeTo(3.0, 1e-9));

    final degrees = {
      for (final r in route.rotationTargets)
        r.rotation.degrees.round(): r.waypointRelativePos
    };
    expect(degrees[90], closeTo(0.5, 1e-9));
    expect(degrees.containsKey(45), false);
    expect(degrees[180], closeTo(2.5, 1e-9));
  });

  test('the route ends where the pivot ends, facing its end rotation', () {
    final region = regionWithPivot();
    final route = PivotRoute.build(mainPath(), region);

    expect(
        route.waypoints.last.anchor, region.divertPath!.waypoints.last.anchor);
    expect(route.goalEndState.rotation.degrees, closeTo(-90, 1e-9));
  });

  test('path points inside the slow zone use its speed limit', () {
    final route = PivotRoute.build(mainPath(), regionWithPivot());
    final inZone = route.pathPoints
        .where((p) => p.waypointPos > 0.3 && p.waypointPos < 0.5)
        .toList();
    expect(inZone, isNotEmpty);
    for (final p in inZone) {
      expect(p.constraints.maxVelocityMPS, 1.0);
    }
  });
}
