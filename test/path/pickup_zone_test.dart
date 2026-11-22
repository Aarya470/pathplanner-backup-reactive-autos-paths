import 'dart:convert';
import 'dart:math';

import 'package:file/memory.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:pathplanner/path/obstacle_region.dart';
import 'package:pathplanner/path/pathplanner_path.dart';
import 'package:pathplanner/path/pickup_zone.dart';
import 'package:pathplanner/path/waypoint.dart';
import 'package:pathplanner/util/wpimath/geometry.dart';

bool _inside(List<Translation2d> poly, Translation2d p) {
  bool inside = false;
  for (int i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    final a = poly[i], b = poly[j];
    if ((a.y > p.y) != (b.y > p.y) &&
        p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) {
      inside = !inside;
    }
  }
  return inside;
}

void main() {
  late PathPlannerPath path;

  setUp(() {
    path =
        PathPlannerPath.defaultPath(pathDir: '/paths', fs: MemoryFileSystem());
  });

  test('default zone surrounds the whole region', () {
    final region = ObstacleRegion(
        startPos: 0.1, endPos: 0.9, trigger: RegionTrigger.gamePiece);
    final zone = PickupZone.defaultZone(path, region);
    expect(zone.length, 4);
    for (final p in PickupZone.regionPoints(path, region)) {
      expect(_inside(zone, p), isTrue, reason: '$p');
    }
  });

  test('default zone corners are within reach', () {
    final region = ObstacleRegion(
        startPos: 0.3, endPos: 0.35, trigger: RegionTrigger.gamePiece);
    for (final c in PickupZone.defaultZone(path, region)) {
      expect(PickupZone.clampCorner(path, region, c), c);
    }
  });

  test('reach grows with the region and is capped', () {
    final short = ObstacleRegion(startPos: 0.4, endPos: 0.45);
    final long = ObstacleRegion(startPos: 0, endPos: 1);
    expect(PickupZone.reach(path, short), PickupZone.minReach);
    expect(PickupZone.reach(path, long),
        greaterThanOrEqualTo(PickupZone.reach(path, short)));
    expect(
        PickupZone.reach(path, long), lessThanOrEqualTo(PickupZone.maxReach));
  });

  test('a corner dragged far away is pulled back to the reach limit', () {
    final region = ObstacleRegion(startPos: 0.2, endPos: 0.6);
    const far = Translation2d(15, 7);
    final clamped = PickupZone.clampCorner(path, region, far);
    final double limit = PickupZone.reach(path, region);
    final num nearest = PickupZone.regionPoints(path, region)
        .map((p) => p.getDistance(clamped))
        .reduce((a, b) => a < b ? a : b);
    expect(nearest, lessThanOrEqualTo(limit + 0.06));
    expect(clamped.getDistance(far), greaterThan(1));
  });

  test('default zone corners are within reach on curvy paths', () {
    final rnd = Random(1);
    for (int k = 0; k < 300; k++) {
      final p = PathPlannerPath.defaultPath(pathDir: '/paths', fs: MemoryFileSystem());
      final pts = [
        for (int j = 0; j < 3; j++)
          Translation2d(2 + rnd.nextDouble() * 10, 1 + rnd.nextDouble() * 6)
      ];
      Translation2d jitter() =>
          Translation2d(rnd.nextDouble() * 4 - 2, rnd.nextDouble() * 4 - 2);
      p.waypoints = [
        for (int j = 0; j < 3; j++)
          Waypoint(
              anchor: pts[j],
              prevControl: j > 0 ? pts[j] + jitter() : null,
              nextControl: j < 2 ? pts[j] + jitter() : null),
      ];
      p.generatePathPoints();
      final s = rnd.nextDouble() * 2;
      final e = s + rnd.nextDouble() * (2 - s);
      final r = ObstacleRegion(startPos: s, endPos: e, trigger: RegionTrigger.gamePiece);
      for (final c in PickupZone.defaultZone(p, r)) {
        expect(PickupZone.clampCorner(p, r, c).getDistance(c), lessThan(1e-6));
      }
    }
  });

  test('a crossed zone is not simple', () {
    const square = [
      Translation2d(0, 0),
      Translation2d(1, 0),
      Translation2d(1, 1),
      Translation2d(0, 1),
    ];
    expect(PickupZone.isSimple(square), isTrue);
    const bowTie = [
      Translation2d(0, 0),
      Translation2d(1, 0),
      Translation2d(0, 1),
      Translation2d(1, 1),
    ];
    expect(PickupZone.isSimple(bowTie), isFalse);
  });

  test('older paths get a zone when loaded, and it is saved', () async {
    final fs = MemoryFileSystem();
    await fs.directory('/paths').create(recursive: true);
    final old = PathPlannerPath.defaultPath(pathDir: '/paths', fs: fs, name: 'Old');
    old.obstacleRegions = [
      ObstacleRegion(startPos: 0.2, endPos: 0.6, trigger: RegionTrigger.gamePiece)
    ];
    final json = old.toJson();
    expect(json['obstacleRegions'][0]['pickupZone'], isNull);
    await fs.file('/paths/Old.path').writeAsString(jsonEncode(json));

    final loaded = await PathPlannerPath.loadAllPathsInDir('/paths', fs);
    expect(loaded.single.obstacleRegions.single.pickupZone, isNotNull);
    await Future.delayed(const Duration(milliseconds: 300));
    final saved = jsonDecode(await fs.file('/paths/Old.path').readAsString());
    expect(saved['obstacleRegions'][0]['pickupZone'], isNotNull);
  });
}
