import 'package:file/memory.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:pathplanner/path/obstacle_region.dart';
import 'package:pathplanner/path/pathplanner_path.dart';
import 'package:pathplanner/util/wpimath/geometry.dart';

void main() {
  test('Region without a diverge path round trips', () {
    ObstacleRegion region = ObstacleRegion(
      startPos: 0.5,
      endPos: 1.5,
      obstacleTeam: ObstacleTeam.opponent,
      blockTimeoutSec: 2.0,
      action: RegionAction.continuePath,
    );

    Map<String, dynamic> json = region.toJson();
    expect(json.containsKey('divertPath'), false);

    ObstacleRegion parsed = ObstacleRegion.fromJson(json,
        pathsDir: '/paths', fs: MemoryFileSystem());
    expect(parsed, region);
    expect(parsed.divertPath, isNull);
  });

  test('Divert region keeps its diverge path and marks it nested', () {
    final fs = MemoryFileSystem();
    PathPlannerPath diverge = PathPlannerPath.defaultPath(
      pathDir: '/paths',
      fs: fs,
      name: 'diverge',
    );
    diverge.addWaypoint(const Translation2d(1.0, 1.0));
    diverge.addWaypoint(const Translation2d(3.0, 1.0));

    ObstacleRegion region = ObstacleRegion(
      startPos: 0,
      endPos: 1,
      action: RegionAction.divert,
      divertPath: diverge,
    );

    Map<String, dynamic> json = region.toJson();
    expect(json['action'], 'divert');
    expect(json['divertPath'], isNotNull);

    ObstacleRegion parsed =
        ObstacleRegion.fromJson(json, pathsDir: '/paths', fs: fs);
    expect(parsed.action, RegionAction.divert);
    expect(parsed.divertPath, isNotNull);
    expect(parsed.divertPath!.isNested, true);
    expect(parsed.divertPath!.waypoints.length, diverge.waypoints.length);
  });

  test('Missing timeout stays null, so the region diverges right away', () {
    ObstacleRegion region = ObstacleRegion(
      startPos: 0,
      endPos: 1,
      action: RegionAction.divert,
    );
    expect(region.blockTimeoutSec, isNull);

    ObstacleRegion parsed = ObstacleRegion.fromJson(region.toJson(),
        pathsDir: '/paths', fs: MemoryFileSystem());
    expect(parsed.blockTimeoutSec, isNull);
  });

  test('Unknown action and team fall back to defaults', () {
    expect(RegionAction.fromName('bogus'), RegionAction.continuePath);
    expect(ObstacleTeam.fromName(null), ObstacleTeam.any);
    expect(ObstacleTeam.fromName('teammate'), ObstacleTeam.teammate);
  });

  test('Clone is equal but independent', () {
    ObstacleRegion region = ObstacleRegion(startPos: 0.2, endPos: 0.8);
    ObstacleRegion copy = region.clone();
    expect(copy, region);
    copy.endPos = 1.0;
    expect(copy, isNot(region));
    expect(region.endPos, 0.8);
  });

  test('Game piece region round trips its trigger and distance', () {
    ObstacleRegion region = ObstacleRegion(
      startPos: 0.2,
      endPos: 0.8,
      action: RegionAction.continuePath,
      trigger: RegionTrigger.gamePiece,
      pieceMaxDistance: 1.5,
    );

    Map<String, dynamic> json = region.toJson();
    expect(json['trigger'], 'gamePiece');
    expect(json['pieceMaxDistance'], 1.5);

    ObstacleRegion parsed = ObstacleRegion.fromJson(json,
        pathsDir: '/paths', fs: MemoryFileSystem());
    expect(parsed, region);
    expect(parsed.clone(), region);
  });

  test('Older files without a trigger load as robot regions', () {
    ObstacleRegion parsed = ObstacleRegion.fromJson({
      'startPos': 0,
      'endPos': 1,
      'obstacleTeam': 'opponent',
      'action': 'divert',
    }, pathsDir: '/paths', fs: MemoryFileSystem());
    expect(parsed.trigger, RegionTrigger.robot);
    expect(parsed.pieceMaxDistance, ObstacleRegion.defaultPieceMaxDistance);
  });

  test('Pickup zone round trips through the path file', () {
    ObstacleRegion region = ObstacleRegion(
      startPos: 0.2,
      endPos: 0.8,
      trigger: RegionTrigger.gamePiece,
      pickupZone: const [
        Translation2d(1.0, 1.0),
        Translation2d(3.0, 1.0),
        Translation2d(3.0, 3.0),
        Translation2d(1.0, 3.0),
      ],
    );

    Map<String, dynamic> json = region.toJson();
    expect(json['pickupZone'], [
      {'x': 1.0, 'y': 1.0},
      {'x': 3.0, 'y': 1.0},
      {'x': 3.0, 'y': 3.0},
      {'x': 1.0, 'y': 3.0},
    ]);

    ObstacleRegion parsed = ObstacleRegion.fromJson(json,
        pathsDir: '/paths', fs: MemoryFileSystem());
    expect(parsed.pickupZone, region.pickupZone);
    expect(parsed, region);
  });

  test('Regions without a zone do not save one', () {
    ObstacleRegion region = ObstacleRegion(
        startPos: 0, endPos: 1, trigger: RegionTrigger.gamePiece);
    expect(region.toJson().containsKey('pickupZone'), false);
    expect(
        ObstacleRegion.fromJson(region.toJson(),
                pathsDir: '/paths', fs: MemoryFileSystem())
            .pickupZone,
        isNull);
  });

  test('A copied zone can be changed without changing the original', () {
    ObstacleRegion region = ObstacleRegion(
        startPos: 0,
        endPos: 1,
        trigger: RegionTrigger.gamePiece,
        pickupZone: ObstacleRegion.squareZone(const Translation2d(2, 2)));
    ObstacleRegion copy = region.clone();
    copy.pickupZone![0] = const Translation2d(9, 9);
    expect(region.pickupZone![0], isNot(const Translation2d(9, 9)));
    expect(copy == region, false);
  });

  test('A new zone is a square around the center', () {
    final zone = ObstacleRegion.squareZone(const Translation2d(2, 3), 0.5);
    expect(zone, const [
      Translation2d(1.5, 2.5),
      Translation2d(2.5, 2.5),
      Translation2d(2.5, 3.5),
      Translation2d(1.5, 3.5),
    ]);
  });
}
