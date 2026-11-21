import 'package:file/file.dart';
import 'package:flutter/foundation.dart';
import 'package:pathplanner/path/pathplanner_path.dart';
import 'package:pathplanner/util/wpimath/geometry.dart';

/// Which robots count as obstacles for a region
enum ObstacleTeam {
  opponent,
  teammate,
  any;

  static ObstacleTeam fromName(String? name) {
    for (final team in ObstacleTeam.values) {
      if (team.name == name) return team;
    }
    return ObstacleTeam.any;
  }
}

/// What the robot does when an obstacle stays in a region for the full timeout
enum RegionAction {
  /// Ignore the obstacle and keep following the main path
  continuePath,

  /// Leave the main path where the robot is and follow the diverge path
  divert;

  static RegionAction fromName(String? name) {
    for (final action in RegionAction.values) {
      if (action.name == name) return action;
    }
    return RegionAction.continuePath;
  }
}

/// What a region watches for
enum RegionTrigger {
  /// Another robot in the way of the path
  robot,

  /// A game piece close to the robot, to go pick up
  gamePiece;

  static RegionTrigger fromName(String? name) {
    for (final trigger in RegionTrigger.values) {
      if (trigger.name == name) return trigger;
    }
    return RegionTrigger.robot;
  }
}

/// A part of a path that is watched for another robot or for a game piece.
///
/// [startPos] and [endPos] use the same scale as rotation targets: 0 is the first
/// waypoint and the last waypoint is the number of waypoints minus one.
class ObstacleRegion {
  num startPos;
  num endPos;
  ObstacleTeam obstacleTeam;

  /// Seconds to stop and wait before deciding. Null means decide right away.
  double? blockTimeoutSec;
  RegionAction action;

  /// The path followed after a divert. Only used when [action] is [RegionAction.divert].
  PathPlannerPath? divertPath;

  /// What the region watches for
  RegionTrigger trigger;

  /// Game piece regions: how close a piece must be to the robot to go get it, meters
  double pieceMaxDistance;

  static const double defaultPieceMaxDistance = 2.0;

  /// Game piece regions: corners of the pickup zone in field meters, in order around the edge.
  /// When set, a piece is picked up only if it is inside this zone, and [pieceMaxDistance] is not
  /// used. Null means no zone.
  List<Translation2d>? pickupZone;

  /// Half the width of a new pickup zone, meters
  static const double defaultZoneHalfSize = 0.75;

  ObstacleRegion({
    required this.startPos,
    required this.endPos,
    this.obstacleTeam = ObstacleTeam.any,
    this.blockTimeoutSec,
    this.action = RegionAction.continuePath,
    this.divertPath,
    this.trigger = RegionTrigger.robot,
    this.pieceMaxDistance = defaultPieceMaxDistance,
    this.pickupZone,
  });

  /// A square pickup zone centered on [center]
  static List<Translation2d> squareZone(Translation2d center,
      [double halfSize = defaultZoneHalfSize]) {
    return [
      Translation2d(center.x - halfSize, center.y - halfSize),
      Translation2d(center.x + halfSize, center.y - halfSize),
      Translation2d(center.x + halfSize, center.y + halfSize),
      Translation2d(center.x - halfSize, center.y + halfSize),
    ];
  }

  /// Parse one entry from a path file. [pathsDir] and [fs] are the same values the
  /// parent path was loaded with, so the diverge path can be edited like any other path.
  ObstacleRegion.fromJson(Map<String, dynamic> json,
      {required String pathsDir, required FileSystem fs})
      : startPos = json['startPos'] ?? 0,
        endPos = json['endPos'] ?? 0,
        obstacleTeam = ObstacleTeam.fromName(json['obstacleTeam']),
        blockTimeoutSec = json['blockTimeoutSec'] == null
            ? null
            : (json['blockTimeoutSec'] as num).toDouble(),
        action = RegionAction.fromName(json['action']),
        divertPath = json['divertPath'] == null
            ? null
            : _nestedPath(json['divertPath'], pathsDir, fs),
        trigger = RegionTrigger.fromName(json['trigger']),
        pieceMaxDistance = json['pieceMaxDistance'] == null
            ? defaultPieceMaxDistance
            : (json['pieceMaxDistance'] as num).toDouble(),
        pickupZone = _zoneFromJson(json['pickupZone']);

  static List<Translation2d>? _zoneFromJson(dynamic value) {
    if (value is! List || value.length < 3) return null;
    return [
      for (final corner in value)
        Translation2d.fromJson(Map<String, dynamic>.from(corner)),
    ];
  }

  ObstacleRegion clone() {
    return ObstacleRegion(
      startPos: startPos,
      endPos: endPos,
      obstacleTeam: obstacleTeam,
      blockTimeoutSec: blockTimeoutSec,
      action: action,
      divertPath: _copyDivertPath(),
      trigger: trigger,
      pieceMaxDistance: pieceMaxDistance,
      pickupZone: pickupZone == null ? null : List.of(pickupZone!),
    );
  }

  static PathPlannerPath _nestedPath(
      Map<String, dynamic> json, String pathsDir, FileSystem fs) {
    final path = PathPlannerPath.fromJson(Map<String, dynamic>.from(json),
        json['name'] ?? 'Diverge Path', pathsDir, fs);
    path.isNested = true;
    return path;
  }

  PathPlannerPath? _copyDivertPath() {
    if (divertPath == null) return null;
    final copy = divertPath!.duplicate(divertPath!.name);
    copy.isNested = true;
    return copy;
  }

  Map<String, dynamic> toJson() {
    return {
      'startPos': startPos,
      'endPos': endPos,
      'obstacleTeam': obstacleTeam.name,
      'blockTimeoutSec': blockTimeoutSec,
      'action': action.name,
      if (divertPath != null) 'divertPath': divertPath!.toJson(),
      'trigger': trigger.name,
      'pieceMaxDistance': pieceMaxDistance,
      if (pickupZone != null)
        'pickupZone': [for (final c in pickupZone!) c.toJson()],
    };
  }

  @override
  bool operator ==(Object other) =>
      other is ObstacleRegion &&
      other.runtimeType == runtimeType &&
      other.startPos == startPos &&
      other.endPos == endPos &&
      other.obstacleTeam == obstacleTeam &&
      other.blockTimeoutSec == blockTimeoutSec &&
      other.action == action &&
      other.trigger == trigger &&
      other.pieceMaxDistance == pieceMaxDistance &&
      listEquals(other.pickupZone, pickupZone);

  @override
  int get hashCode => Object.hash(startPos, endPos, obstacleTeam,
      blockTimeoutSec, action, trigger, pieceMaxDistance,
      pickupZone == null ? null : Object.hashAll(pickupZone!));
}
