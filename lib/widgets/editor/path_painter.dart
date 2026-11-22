import 'dart:convert';
import 'dart:math';
import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:pathplanner/path/choreo_path.dart';
import 'package:pathplanner/path/obstacle_region.dart';
import 'package:pathplanner/path/point_towards_zone.dart';
import 'package:pathplanner/path/rotation_target.dart';
import 'package:pathplanner/robot_features/feature.dart';
import 'package:pathplanner/trajectory/config.dart';
import 'package:pathplanner/trajectory/trajectory.dart';
import 'package:pathplanner/path/pathplanner_path.dart';
import 'package:pathplanner/path/waypoint.dart';
import 'package:pathplanner/util/prefs.dart';
import 'package:pathplanner/util/wpimath/geometry.dart';
import 'package:pathplanner/widgets/field_image.dart';
import 'package:pathplanner/util/path_painter_util.dart';
import 'package:shared_preferences/shared_preferences.dart';

class PathPainter extends CustomPainter {
  final ColorScheme colorScheme;
  final List<PathPlannerPath> paths;
  final List<ChoreoPath> choreoPaths;
  final FieldImage fieldImage;
  final bool simple;
  final bool hideOtherPathsOnHover;
  final String? hoveredPath;
  final int? hoveredWaypoint;
  final int? selectedWaypoint;
  final int? hoveredZone;
  final int? selectedZone;
  final int? hoveredPointZone;
  final int? selectedPointZone;
  final int? hoveredRegion;
  final int? selectedRegion;
  final PathPlannerPath? ghostPath;
  final ObstacleRegion? highlightRegion;
  final ObstacleRegion? wholePivot;
  final ObstacleRegion? continueRegion;

  /// The full route shown in Pivot view (main path up to the break-off, then the pivot path).
  /// Its waypoints, rotations, markers and end outline are drawn instead of the main path's.
  final PathPlannerPath? pivotRoute;

  /// Waypoints up to this index in [pivotRoute] are the main path's own and can be highlighted
  final int pivotSplitIndex;
  final int? hoveredRotTarget;
  final int? selectedRotTarget;
  final int? hoveredMarker;
  final int? selectedMarker;
  final PathPlannerTrajectory? simulatedPath;
  final SharedPreferences prefs;
  final PathPlannerPath? optimizedPath;

  late final RobotConfig robotConfig;
  late final num robotRadius;
  Animation<num>? previewTime;
  final List<Feature> robotFeatures = [];

  static double scale = 1;

  PathPainter({
    required this.colorScheme,
    required this.paths,
    this.choreoPaths = const [],
    required this.fieldImage,
    this.simple = false,
    this.hideOtherPathsOnHover = false,
    this.hoveredPath,
    this.hoveredWaypoint,
    this.selectedWaypoint,
    this.hoveredZone,
    this.selectedZone,
    this.hoveredPointZone,
    this.selectedPointZone,
    this.hoveredRegion,
    this.selectedRegion,
    this.ghostPath,
    this.highlightRegion,
    this.wholePivot,
    this.continueRegion,
    this.pivotRoute,
    this.pivotSplitIndex = -1,
    this.hoveredRotTarget,
    this.selectedRotTarget,
    this.hoveredMarker,
    this.selectedMarker,
    this.simulatedPath,
    Animation<double>? animation,
    required this.prefs,
    this.optimizedPath,
  }) : super(repaint: animation) {
    robotConfig = RobotConfig.fromPrefs(prefs);
    robotRadius = sqrt((robotConfig.bumperSize.width *
                robotConfig.bumperSize.width) +
            (robotConfig.bumperSize.height * robotConfig.bumperSize.height)) /
        2.0;

    for (String featureJson in prefs.getStringList(PrefsKeys.robotFeatures) ??
        Defaults.robotFeatures) {
      try {
        robotFeatures.add(Feature.fromJson(jsonDecode(featureJson))!);
      } catch (_) {
        // Ignore and skip loading this feature
      }
    }

    if (simulatedPath != null && animation != null) {
      previewTime =
          Tween<num>(begin: 0, end: simulatedPath!.states.last.timeSeconds)
              .animate(animation);
    }
  }

  @override
  void paint(Canvas canvas, Size size) {
    scale = size.width / fieldImage.defaultSize.width;

    _paintGrid(
        canvas, size, prefs.getBool(PrefsKeys.showGrid) ?? Defaults.showGrid);

    for (int i = 0; i < paths.length; i++) {
      if (hideOtherPathsOnHover &&
          hoveredPath != null &&
          hoveredPath != paths[i].name) {
        continue;
      }

      if (!simple) {
        _paintRadius(paths[i], canvas, scale);
      }

      if (paths[i].isNested) {
        if (ghostPath != null) {
          _paintGhostLine(ghostPath!, canvas, scale);
          if (highlightRegion != null) {
            _paintRegionSpan(ghostPath!, highlightRegion!, canvas, scale, Colors.yellow);
          }
        }
        _paintDashedPath(paths[i], canvas, colorScheme.secondary, scale);
        if (paths[i].waypoints.isNotEmpty) {
          canvas.drawCircle(
              PathPainterUtil.pointToPixelOffset(
                  paths[i].waypoints.first.anchor, scale, fieldImage),
              9,
              Paint()..color = Colors.green);
        }
        if (paths[i].waypoints.length > 1) {
          canvas.drawCircle(
              PathPainterUtil.pointToPixelOffset(
                  paths[i].waypoints.last.anchor, scale, fieldImage),
              9,
              Paint()..color = Colors.red);
        }
      } else if (wholePivot != null && wholePivot!.divertPath != null) {
        _paintPivotRoute(paths[i], wholePivot!, canvas, scale);
        final PathPlannerPath? route = pivotRoute;
        if (route != null && route.waypoints.isNotEmpty) {
          if (robotConfig.holonomic) {
            _paintRotations(route, canvas, scale);
          }
          _paintMarkers(route, canvas);
          for (int w = 0; w < route.waypoints.length; w++) {
            _paintWaypoint(route, canvas, scale, w,
                highlight: w <= pivotSplitIndex);
          }
          _paintPointZonePositions(route, canvas, scale);
          continue;
        }
      } else {
        _paintPathPoints(
            paths[i],
            canvas,
            (hoveredPath == paths[i].name)
                ? Colors.orange
                : colorScheme.secondary);
        _paintRegions(paths[i], canvas, scale);
        _paintPickupZones(paths[i], canvas, scale);
        final pause = continueRegion;
        if (pause != null && pause.blockTimeoutSec != null) {
          canvas.drawCircle(
              PathPainterUtil.pointToPixelOffset(
                  paths[i].samplePath(pause.startPos), scale, fieldImage),
              8,
              Paint()..color = Colors.yellow);
        }
      }

      if (robotConfig.holonomic) {
        _paintRotations(paths[i], canvas, scale);
      }

      _paintMarkers(paths[i], canvas);

      if (!simple) {
        for (int w = 0; w < paths[i].waypoints.length; w++) {
          _paintWaypoint(paths[i], canvas, scale, w);
        }
      } else {
        _paintWaypoint(paths[i], canvas, scale, 0);
        _paintWaypoint(paths[i], canvas, scale, paths[i].waypoints.length - 1);
      }

      _paintPointZonePositions(paths[i], canvas, scale);
    }

    for (int i = 0; i < choreoPaths.length; i++) {
      if (hideOtherPathsOnHover &&
          hoveredPath != null &&
          hoveredPath != choreoPaths[i].name) {
        continue;
      }

      if (choreoPaths[i].trajectory.states.isEmpty) {
        continue;
      }

      _paintTrajectory(
          choreoPaths[i].trajectory,
          canvas,
          (hoveredPath == choreoPaths[i].name)
              ? Colors.orange
              : colorScheme.secondary);
      _paintChoreoWaypoint(
          choreoPaths[i].trajectory.states.first, canvas, Colors.green, scale);
      _paintChoreoWaypoint(
          choreoPaths[i].trajectory.states.last, canvas, Colors.red, scale);
      _paintChoreoMarkers(choreoPaths[i], canvas);
    }

    if (optimizedPath != null) {
      _paintPathPoints(optimizedPath!, canvas, Colors.deepPurpleAccent, 4.0);
    }

    for (int i = 1; i < paths.length; i++) {
      // Paint warnings between breaks in paths
      Translation2d prevPathEnd = paths[i - 1].pathPoints.last.position;
      Translation2d pathStart = paths[i].pathPoints.first.position;

      if (prevPathEnd.getDistance(pathStart) >= 0.25) {
        _paintBreakWarning(prevPathEnd, pathStart, canvas, scale);
      }
    }

    for (int i = 1; i < choreoPaths.length; i++) {
      // Paint warnings between breaks in paths
      Translation2d prevPathEnd =
          choreoPaths[i - 1].trajectory.states.last.pose.translation;
      Translation2d pathStart =
          choreoPaths[i].trajectory.states.first.pose.translation;

      if (prevPathEnd.getDistance(pathStart) >= 0.25) {
        _paintBreakWarning(prevPathEnd, pathStart, canvas, scale);
      }
    }

    if (prefs.getBool(PrefsKeys.showStates) ?? Defaults.showStates) {
      _paintTrajectoryStates(simulatedPath, canvas);
    }

    if (previewTime != null) {
      TrajectoryState state = simulatedPath!.sample(previewTime!.value);
      Rotation2d rotation = state.pose.rotation;

      if (robotConfig.holonomic && state.moduleStates.isNotEmpty) {
        // Calculate the module positions based off of the robot position
        // so they don't move relative to the robot when interpolating
        // between trajectory states
        List<Pose2d> modPoses = [
          Pose2d(
              state.pose.translation +
                  robotConfig.moduleLocations[0].rotateBy(rotation),
              state.moduleStates[0].fieldAngle),
          Pose2d(
              state.pose.translation +
                  robotConfig.moduleLocations[1].rotateBy(rotation),
              state.moduleStates[1].fieldAngle),
          Pose2d(
              state.pose.translation +
                  robotConfig.moduleLocations[2].rotateBy(rotation),
              state.moduleStates[2].fieldAngle),
          Pose2d(
              state.pose.translation +
                  robotConfig.moduleLocations[3].rotateBy(rotation),
              state.moduleStates[3].fieldAngle),
        ];
        PathPainterUtil.paintRobotModules(
            modPoses, fieldImage, scale, canvas, colorScheme.primary);
      }

      PathPainterUtil.paintRobotOutline(
        Pose2d(state.pose.translation, rotation),
        fieldImage,
        robotConfig.bumperSize,
        robotConfig.bumperOffset,
        scale,
        canvas,
        colorScheme.primary,
        colorScheme.surfaceContainer,
        robotFeatures,
        showDetails: prefs.getBool(PrefsKeys.showRobotDetails) ??
            Defaults.showRobotDetails,
      );
    }
  }

  @override
  bool shouldRepaint(PathPainter oldDelegate) {
    return true; // This will just be repainted all the time anyways from the animation
  }

  void _paintTrajectoryStates(PathPlannerTrajectory? traj, Canvas canvas) {
    if (traj == null) {
      return;
    }

    var paint = Paint()..style = PaintingStyle.fill;

    num maxVel = 0.0;
    for (TrajectoryState s in traj.states) {
      maxVel = max(
          maxVel, sqrt(pow(s.fieldSpeeds.vx, 2) + pow(s.fieldSpeeds.vy, 2)));
    }

    for (TrajectoryState s in traj.states) {
      num normalizedVel =
          sqrt(pow(s.fieldSpeeds.vx, 2) + pow(s.fieldSpeeds.vy, 2)) / maxVel;
      normalizedVel = normalizedVel.clamp(0.0, 1.0);

      if (normalizedVel <= 0.33) {
        // Lerp between red and orange
        paint.color =
            Color.lerp(Colors.red, Colors.orange, normalizedVel / 0.33)!;
      } else if (normalizedVel <= 0.67) {
        // Lerp between orange and yellow
        paint.color = Color.lerp(
            Colors.orange, Colors.yellow, (normalizedVel - 0.33) / 0.34)!;
      } else {
        // Lerp between yellow and green
        paint.color = Color.lerp(
            Colors.yellow, Colors.green, (normalizedVel - 0.67) / 0.33)!;
      }
      Offset pos = PathPainterUtil.pointToPixelOffset(
          s.pose.translation, scale, fieldImage);
      canvas.drawCircle(pos, 3.0, paint);
    }
  }

  void _paintTrajectory(
      PathPlannerTrajectory traj, Canvas canvas, Color baseColor) {
    var paint = Paint()
      ..style = PaintingStyle.stroke
      ..color = baseColor
      ..strokeWidth = 2;

    Path p = Path();

    Offset start = PathPainterUtil.pointToPixelOffset(
        traj.states.first.pose.translation, scale, fieldImage);
    p.moveTo(start.dx, start.dy);

    for (int i = 1; i < traj.states.length; i++) {
      Offset pos = PathPainterUtil.pointToPixelOffset(
          traj.states[i].pose.translation, scale, fieldImage);

      p.lineTo(pos.dx, pos.dy);
    }

    canvas.drawPath(p, paint);
  }

  void _paintChoreoWaypoint(
      TrajectoryState state, Canvas canvas, Color color, double scale) {
    var paint = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2
      ..color = color;

    // draw anchor point
    paint.style = PaintingStyle.fill;
    canvas.drawCircle(
        PathPainterUtil.pointToPixelOffset(
            state.pose.translation, scale, fieldImage),
        PathPainterUtil.uiPointSizeToPixels(25, scale, fieldImage),
        paint);
    paint.style = PaintingStyle.stroke;
    paint.color = colorScheme.surfaceContainer;
    canvas.drawCircle(
        PathPainterUtil.pointToPixelOffset(
            state.pose.translation, scale, fieldImage),
        PathPainterUtil.uiPointSizeToPixels(25, scale, fieldImage),
        paint);

    // Draw robot
    PathPainterUtil.paintRobotOutline(
        state.pose,
        fieldImage,
        robotConfig.bumperSize,
        robotConfig.bumperOffset,
        scale,
        canvas,
        color.withAlpha(125),
        colorScheme.surfaceContainer,
        robotFeatures);
  }

  void _paintPathPoints(PathPlannerPath path, Canvas canvas, Color baseColor,
      [double strokeWidth = 2.0]) {
    var paint = Paint()
      ..style = PaintingStyle.stroke
      ..color = baseColor
      ..strokeWidth = strokeWidth;

    Path p = Path();

    Offset start = PathPainterUtil.pointToPixelOffset(
        path.pathPoints[0].position, scale, fieldImage);
    p.moveTo(start.dx, start.dy);

    for (int i = 1; i < path.pathPoints.length; i++) {
      Offset pos = PathPainterUtil.pointToPixelOffset(
          path.pathPoints[i].position, scale, fieldImage);

      p.lineTo(pos.dx, pos.dy);
    }

    canvas.drawPath(p, paint);

    if (selectedZone != null) {
      paint.color = Colors.orange;
      paint.strokeWidth = 6;
      p.reset();

      num startPos = path.constraintZones[selectedZone!].minWaypointRelativePos;
      num endPos = path.constraintZones[selectedZone!].maxWaypointRelativePos;

      Offset start = PathPainterUtil.pointToPixelOffset(
          path.samplePath(startPos), scale, fieldImage);
      p.moveTo(start.dx, start.dy);

      for (num t = startPos + 0.05; t <= endPos; t += 0.05) {
        Offset pos = PathPainterUtil.pointToPixelOffset(
            path.samplePath(t), scale, fieldImage);

        p.lineTo(pos.dx, pos.dy);
      }
      Offset end = PathPainterUtil.pointToPixelOffset(
          path.samplePath(endPos), scale, fieldImage);
      p.lineTo(end.dx, end.dy);

      canvas.drawPath(p, paint);
    }

    if (hoveredZone != null && selectedZone != hoveredZone) {
      paint.color = Colors.deepPurpleAccent;
      paint.strokeWidth = 6;
      p.reset();

      num startPos = path.constraintZones[hoveredZone!].minWaypointRelativePos;
      num endPos = path.constraintZones[hoveredZone!].maxWaypointRelativePos;

      Offset start = PathPainterUtil.pointToPixelOffset(
          path.samplePath(startPos), scale, fieldImage);
      p.moveTo(start.dx, start.dy);

      for (num t = startPos + 0.05; t <= endPos; t += 0.05) {
        Offset pos = PathPainterUtil.pointToPixelOffset(
            path.samplePath(t), scale, fieldImage);

        p.lineTo(pos.dx, pos.dy);
      }
      Offset end = PathPainterUtil.pointToPixelOffset(
          path.samplePath(endPos), scale, fieldImage);
      p.lineTo(end.dx, end.dy);

      canvas.drawPath(p, paint);
    }

    if (selectedPointZone != null) {
      paint.color = Colors.orange;
      paint.strokeWidth = 6;
      p.reset();

      num startPos =
          path.pointTowardsZones[selectedPointZone!].minWaypointRelativePos;
      num endPos =
          path.pointTowardsZones[selectedPointZone!].maxWaypointRelativePos;

      Offset start = PathPainterUtil.pointToPixelOffset(
          path.samplePath(startPos), scale, fieldImage);
      p.moveTo(start.dx, start.dy);

      for (num t = startPos + 0.05; t <= endPos; t += 0.05) {
        Offset pos = PathPainterUtil.pointToPixelOffset(
            path.samplePath(t), scale, fieldImage);

        p.lineTo(pos.dx, pos.dy);
      }

      Offset end = PathPainterUtil.pointToPixelOffset(
          path.samplePath(endPos), scale, fieldImage);
      p.lineTo(end.dx, end.dy);

      canvas.drawPath(p, paint);
    }

    if (hoveredPointZone != null && selectedPointZone != hoveredPointZone) {
      paint.color = Colors.deepPurpleAccent;
      paint.strokeWidth = 6;
      p.reset();

      num startPos =
          path.pointTowardsZones[hoveredPointZone!].minWaypointRelativePos;
      num endPos =
          path.pointTowardsZones[hoveredPointZone!].maxWaypointRelativePos;

      Offset start = PathPainterUtil.pointToPixelOffset(
          path.samplePath(startPos), scale, fieldImage);
      p.moveTo(start.dx, start.dy);

      for (num t = startPos + 0.05; t <= endPos; t += 0.05) {
        Offset pos = PathPainterUtil.pointToPixelOffset(
            path.samplePath(t), scale, fieldImage);

        p.lineTo(pos.dx, pos.dy);
      }

      Offset end = PathPainterUtil.pointToPixelOffset(
          path.samplePath(endPos), scale, fieldImage);
      p.lineTo(end.dx, end.dy);

      canvas.drawPath(p, paint);
    }

    if (selectedMarker != null && path.eventMarkers[selectedMarker!].isZoned) {
      paint.color = Colors.orange;
      paint.strokeWidth = 6;
      p.reset();

      num startPos = path.eventMarkers[selectedMarker!].waypointRelativePos;
      num endPos = path.eventMarkers[selectedMarker!].endWaypointRelativePos!;

      Offset start = PathPainterUtil.pointToPixelOffset(
          path.samplePath(startPos), scale, fieldImage);
      p.moveTo(start.dx, start.dy);

      for (num t = startPos + 0.05; t <= endPos; t += 0.05) {
        Offset pos = PathPainterUtil.pointToPixelOffset(
            path.samplePath(t), scale, fieldImage);

        p.lineTo(pos.dx, pos.dy);
      }
      Offset end = PathPainterUtil.pointToPixelOffset(
          path.samplePath(endPos), scale, fieldImage);
      p.lineTo(end.dx, end.dy);

      canvas.drawPath(p, paint);
    }

    if (hoveredMarker != null &&
        hoveredMarker != selectedMarker &&
        path.eventMarkers[hoveredMarker!].isZoned) {
      paint.color = Colors.deepPurpleAccent;
      paint.strokeWidth = 6;
      p.reset();

      num startPos = path.eventMarkers[hoveredMarker!].waypointRelativePos;
      num endPos = path.eventMarkers[hoveredMarker!].endWaypointRelativePos!;

      Offset start = PathPainterUtil.pointToPixelOffset(
          path.samplePath(startPos), scale, fieldImage);
      p.moveTo(start.dx, start.dy);

      for (num t = startPos + 0.05; t <= endPos; t += 0.05) {
        Offset pos = PathPainterUtil.pointToPixelOffset(
            path.samplePath(t), scale, fieldImage);

        p.lineTo(pos.dx, pos.dy);
      }
      Offset end = PathPainterUtil.pointToPixelOffset(
          path.samplePath(endPos), scale, fieldImage);
      p.lineTo(end.dx, end.dy);

      canvas.drawPath(p, paint);
    }
  }

  void _paintMarkers(PathPlannerPath path, Canvas canvas) {
    for (int i = 0; i < path.eventMarkers.length; i++) {
      var position = path.samplePath(path.eventMarkers[i].waypointRelativePos);

      Color markerColor = Colors.grey[700]!;
      Color markerStrokeColor = colorScheme.surfaceContainer;
      if (selectedMarker == i) {
        markerColor = Colors.orange;
      } else if (hoveredMarker == i) {
        markerColor = Colors.deepPurpleAccent;
      }

      Offset markerPos =
          PathPainterUtil.pointToPixelOffset(position, scale, fieldImage);

      PathPainterUtil.paintMarker(
          canvas, markerPos, markerColor, markerStrokeColor);
    }
  }

  void _paintChoreoMarkers(ChoreoPath path, Canvas canvas) {
    for (num timestamp in path.eventMarkerTimes) {
      TrajectoryState s = path.trajectory.sample(timestamp);
      Offset markerPos = PathPainterUtil.pointToPixelOffset(
          s.pose.translation, scale, fieldImage);

      PathPainterUtil.paintMarker(
          canvas, markerPos, Colors.grey[700]!, colorScheme.onSurface);
    }
  }

  /// The whole route for one pivot: the main path up to the diverge start (solid), then the
  /// diverge path (dashed). A yellow marker shows where the robot waits, if the region has a timeout.
  void _paintPivotRoute(PathPlannerPath path, ObstacleRegion region, Canvas canvas, double scale) {
    final PathPlannerPath divert = region.divertPath!;
    int cut = path.pathPoints.length - 1;
    if (divert.waypoints.isNotEmpty) {
      final Translation2d start = divert.waypoints.first.anchor;
      double best = double.infinity;
      for (int i = 0; i < path.pathPoints.length; i++) {
        final double d = path.pathPoints[i].position.getDistance(start).toDouble();
        if (d < best) {
          best = d;
          cut = i;
        }
      }
    }

    final mainPaint = Paint()
      ..style = PaintingStyle.stroke
      ..color = colorScheme.secondary
      ..strokeWidth = 2.0;
    if (cut > 0) {
      Path p = Path();
      Offset first = PathPainterUtil.pointToPixelOffset(
          path.pathPoints[0].position, scale, fieldImage);
      p.moveTo(first.dx, first.dy);
      for (int i = 1; i <= cut; i++) {
        Offset o = PathPainterUtil.pointToPixelOffset(
            path.pathPoints[i].position, scale, fieldImage);
        p.lineTo(o.dx, o.dy);
      }
      canvas.drawPath(p, mainPaint);
    }

    _paintDashedPath(divert, canvas, colorScheme.tertiary, scale);

    if (region.blockTimeoutSec != null) {
      canvas.drawCircle(
          PathPainterUtil.pointToPixelOffset(
              path.samplePath(region.startPos), scale, fieldImage),
          8,
          Paint()..color = Colors.yellow);
    }
  }

  /// The main path, drawn faint and solid, so a diverge path can be drawn against it
  void _paintGhostLine(PathPlannerPath path, Canvas canvas, double scale) {
    if (path.pathPoints.length < 2) return;
    final paint = Paint()
      ..style = PaintingStyle.stroke
      ..color = colorScheme.secondary.withAlpha(90)
      ..strokeWidth = 4;
    Path p = Path();
    Offset start = PathPainterUtil.pointToPixelOffset(
        path.pathPoints[0].position, scale, fieldImage);
    p.moveTo(start.dx, start.dy);
    for (int i = 1; i < path.pathPoints.length; i++) {
      Offset o = PathPainterUtil.pointToPixelOffset(
          path.pathPoints[i].position, scale, fieldImage);
      p.lineTo(o.dx, o.dy);
    }
    canvas.drawPath(p, paint);
  }

  /// Draw a path as dashed segments, so diverge paths read differently from the main path
  void _paintDashedPath(
      PathPlannerPath path, Canvas canvas, Color color, double scale) {
    if (path.pathPoints.length < 2) return;
    final paint = Paint()
      ..style = PaintingStyle.stroke
      ..color = color
      ..strokeWidth = 2.5;
    const double dashPx = 5;
    const double gapPx = 4;

    final List<Offset> pts = [
      for (final p in path.pathPoints)
        PathPainterUtil.pointToPixelOffset(p.position, scale, fieldImage),
    ];

    bool on = true;
    double remaining = dashPx;
    for (int i = 0; i + 1 < pts.length; i++) {
      final Offset a = pts[i];
      final Offset b = pts[i + 1];
      final double segLen = (b - a).distance;
      if (segLen == 0) continue;
      double t = 0;
      while (t < segLen) {
        final double step = min(remaining, segLen - t);
        if (on) {
          canvas.drawLine(a + (b - a) * (t / segLen),
              a + (b - a) * ((t + step) / segLen), paint);
        }
        t += step;
        remaining -= step;
        if (remaining <= 1e-9) {
          on = !on;
          remaining = on ? dashPx : gapPx;
        }
      }
    }
  }

  /// Highlight each region on the main path in yellow.
  void _paintRegions(PathPlannerPath path, Canvas canvas, double scale) {
    // Every region gets a faint yellow band so it is always visible on the field. The selected or
    // hovered one is drawn thick and bright, with a dark outline and end caps, so it stands out
    // on any field image.
    for (int i = 0; i < path.obstacleRegions.length; i++) {
      final region = path.obstacleRegions[i];
      final bool active = i == selectedRegion || i == hoveredRegion;
      if (active) continue;
      _paintRegionSpan(path, region, canvas, scale, Colors.yellow.withAlpha(190),
          width: 6);
    }
    for (int i = 0; i < path.obstacleRegions.length; i++) {
      final region = path.obstacleRegions[i];
      final bool active = i == selectedRegion || i == hoveredRegion;
      if (!active) continue;
      _paintRegionSpan(path, region, canvas, scale, Colors.black.withAlpha(170),
          width: 14);
      _paintRegionSpan(path, region, canvas, scale, Colors.yellow, width: 9);
      for (final pos in [region.startPos, region.endPos]) {
        final o = PathPainterUtil.pointToPixelOffset(
            path.samplePath(pos), scale, fieldImage);
        canvas.drawCircle(o, 9, Paint()..color = Colors.yellow);
        canvas.drawCircle(
            o,
            9,
            Paint()
              ..style = PaintingStyle.stroke
              ..strokeWidth = 2
              ..color = Colors.black);
      }
    }
  }

  /// Size of a pickup zone corner handle, in UI points
  static const double zoneCornerSize = 22;

  /// Draw the pickup zone of each game piece region, with a draggable handle on each corner. The
  /// selected or hovered region's zone is drawn brighter, with bigger handles.
  void _paintPickupZones(PathPlannerPath path, Canvas canvas, double scale) {
    for (int i = 0; i < path.obstacleRegions.length; i++) {
      final region = path.obstacleRegions[i];
      final zone = region.pickupZone;
      if (region.trigger != RegionTrigger.gamePiece || zone == null || zone.length < 3) {
        continue;
      }
      final bool active = i == selectedRegion || i == hoveredRegion;
      final corners = [
        for (final c in zone) PathPainterUtil.pointToPixelOffset(c, scale, fieldImage)
      ];
      final outline = Path()..addPolygon(corners, true);

      canvas.drawPath(
          outline,
          Paint()
            ..style = PaintingStyle.fill
            ..color = Colors.orange.withAlpha(active ? 80 : 45));
      canvas.drawPath(
          outline,
          Paint()
            ..style = PaintingStyle.stroke
            ..strokeWidth = active ? 4 : 3
            ..color = Colors.orange.withAlpha(active ? 255 : 200));

      final double r = PathPainterUtil.uiPointSizeToPixels(
          (active ? zoneCornerSize : zoneCornerSize * 0.75) / 2, scale, fieldImage);
      for (final c in corners) {
        canvas.drawCircle(c, r, Paint()..color = Colors.orange);
        canvas.drawCircle(
            c,
            r,
            Paint()
              ..style = PaintingStyle.stroke
              ..strokeWidth = 2
              ..color = active ? Colors.white : Colors.black);
      }
    }
  }

  /// Draw the part of a path that a region covers, in the given color
  void _paintRegionSpan(PathPlannerPath path, ObstacleRegion region, Canvas canvas,
      double scale, Color color, {double width = 6}) {
    if (region.endPos <= region.startPos) return;
    final highlight = Paint()
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.round
      ..strokeJoin = StrokeJoin.round
      ..color = color
      ..strokeWidth = width;
    Path p = Path();
    Offset start = PathPainterUtil.pointToPixelOffset(
        path.samplePath(region.startPos), scale, fieldImage);
    p.moveTo(start.dx, start.dy);
    for (num t = region.startPos + 0.05; t < region.endPos; t += 0.05) {
      Offset o = PathPainterUtil.pointToPixelOffset(
          path.samplePath(t), scale, fieldImage);
      p.lineTo(o.dx, o.dy);
    }
    Offset end = PathPainterUtil.pointToPixelOffset(
        path.samplePath(region.endPos), scale, fieldImage);
    p.lineTo(end.dx, end.dy);
    canvas.drawPath(p, highlight);
  }

  void _paintPointZonePositions(
      PathPlannerPath path, Canvas canvas, double scale) {
    if (selectedPointZone != null) {
      final paint = Paint()
        ..style = PaintingStyle.fill
        ..color = Colors.orange
        ..strokeWidth = 3;

      PointTowardsZone z = path.pointTowardsZones[selectedPointZone!];
      final location = PathPainterUtil.pointToPixelOffset(
          z.fieldPosition, scale, fieldImage);

      canvas.drawCircle(location,
          PathPainterUtil.uiPointSizeToPixels(25, scale, fieldImage), paint);

      paint.style = PaintingStyle.stroke;
      canvas.drawCircle(location,
          PathPainterUtil.uiPointSizeToPixels(40, scale, fieldImage), paint);
    }

    if (hoveredPointZone != null && hoveredPointZone != selectedPointZone) {
      final paint = Paint()
        ..style = PaintingStyle.fill
        ..color = Colors.deepPurpleAccent
        ..strokeWidth = 3;

      PointTowardsZone z = path.pointTowardsZones[hoveredPointZone!];
      final location = PathPainterUtil.pointToPixelOffset(
          z.fieldPosition, scale, fieldImage);

      canvas.drawCircle(location,
          PathPainterUtil.uiPointSizeToPixels(25, scale, fieldImage), paint);

      paint.style = PaintingStyle.stroke;
      canvas.drawCircle(location,
          PathPainterUtil.uiPointSizeToPixels(40, scale, fieldImage), paint);
    }
  }

  void _paintRotations(PathPlannerPath path, Canvas canvas, double scale) {
    for (int i = 0; i < path.pathPoints.length - 1; i++) {
      if (path.pathPoints[i].rotationTarget != null &&
          path.pathPoints[i].rotationTarget!.displayInEditor) {
        RotationTarget target = path.pathPoints[i].rotationTarget!;
        Color rotationColor = Colors.grey[700]!;
        if (selectedRotTarget != null &&
            path.rotationTargets[selectedRotTarget!] == target) {
          rotationColor = Colors.orange;
        } else if (hoveredRotTarget != null &&
            path.rotationTargets[hoveredRotTarget!] == target) {
          rotationColor = Colors.deepPurpleAccent;
        }

        PathPainterUtil.paintRobotOutline(
            Pose2d(path.pathPoints[i].position, target.rotation),
            fieldImage,
            robotConfig.bumperSize,
            robotConfig.bumperOffset,
            scale,
            canvas,
            rotationColor,
            colorScheme.surfaceContainer,
            robotFeatures);
      }
    }

    PathPainterUtil.paintRobotOutline(
        Pose2d(path.waypoints.first.anchor, path.idealStartingState.rotation),
        fieldImage,
        robotConfig.bumperSize,
        robotConfig.bumperOffset,
        scale,
        canvas,
        Colors.green.withAlpha(125),
        colorScheme.surfaceContainer,
        robotFeatures);

    PathPainterUtil.paintRobotOutline(
        Pose2d(path.waypoints[path.waypoints.length - 1].anchor,
            path.goalEndState.rotation),
        fieldImage,
        robotConfig.bumperSize,
        robotConfig.bumperOffset,
        scale,
        canvas,
        Colors.red.withAlpha(125),
        colorScheme.surfaceContainer,
        robotFeatures);
  }

  void _paintBreakWarning(Translation2d prevPathEnd, Translation2d pathStart,
      Canvas canvas, double scale) {
    var paint = Paint()
      ..style = PaintingStyle.stroke
      ..color = Colors.yellow[800]!
      ..strokeWidth = 3;

    final p1 =
        PathPainterUtil.pointToPixelOffset(prevPathEnd, scale, fieldImage);
    final p2 = PathPainterUtil.pointToPixelOffset(pathStart, scale, fieldImage);
    final distance = (p2 - p1).distance;
    final normalizedPattern = [7, 5].map((width) => width / distance).toList();
    final points = <Offset>[];
    double t = 0.0;
    int i = 0;
    while (t < 1.0) {
      points.add(Offset.lerp(p1, p2, t)!);
      t += normalizedPattern[i++];
      points.add(Offset.lerp(p1, p2, t.clamp(0.0, 1.0))!);
      t += normalizedPattern[i++];
      i %= normalizedPattern.length;
    }
    canvas.drawPoints(PointMode.lines, points, paint);

    Offset middle = Offset.lerp(p1, p2, 0.5)!;

    const IconData warningIcon = Icons.warning_rounded;

    TextPainter textPainter = TextPainter(
      textDirection: TextDirection.ltr,
      text: TextSpan(
        text: String.fromCharCode(warningIcon.codePoint),
        style: TextStyle(
          fontSize: 40,
          color: Colors.yellow[700]!,
          fontFamily: warningIcon.fontFamily,
        ),
      ),
    );

    TextPainter textStrokePainter = TextPainter(
      textDirection: TextDirection.ltr,
      text: TextSpan(
        text: String.fromCharCode(warningIcon.codePoint),
        style: TextStyle(
          fontSize: 40,
          fontFamily: warningIcon.fontFamily,
          foreground: Paint()
            ..style = PaintingStyle.stroke
            ..strokeWidth = 0.5
            ..color = colorScheme.surfaceContainer,
        ),
      ),
    );

    textPainter.layout();
    textStrokePainter.layout();

    textPainter.paint(canvas, middle - const Offset(20, 25));
    textStrokePainter.paint(canvas, middle - const Offset(20, 25));
  }

  void _paintRadius(PathPlannerPath path, Canvas canvas, double scale) {
    if (selectedWaypoint != null) {
      var paint = Paint()
        ..style = PaintingStyle.stroke
        ..color = colorScheme.surfaceContainerHighest
        ..strokeWidth = 2;

      canvas.drawCircle(
          PathPainterUtil.pointToPixelOffset(
              path.waypoints[selectedWaypoint!].anchor, scale, fieldImage),
          PathPainterUtil.metersToPixels(
              robotRadius.toDouble(), scale, fieldImage),
          paint);
    }
  }

  void _paintWaypoint(
      PathPlannerPath path, Canvas canvas, double scale, int waypointIdx,
      {bool highlight = true}) {
    final int? selectedWaypoint = highlight ? this.selectedWaypoint : null;
    final int? hoveredWaypoint = highlight ? this.hoveredWaypoint : null;
    var paint = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2;

    if (waypointIdx == selectedWaypoint) {
      paint.color = Colors.orange;
    } else if (waypointIdx == hoveredWaypoint) {
      paint.color = Colors.deepPurpleAccent;
    } else {
      paint.color = Colors.grey[700]!;
    }

    Waypoint waypoint = path.waypoints[waypointIdx];

    if (!simple) {
      //draw control point lines
      if (waypoint.nextControl != null) {
        canvas.drawLine(
            PathPainterUtil.pointToPixelOffset(
                waypoint.anchor, scale, fieldImage),
            PathPainterUtil.pointToPixelOffset(
                waypoint.nextControl!, scale, fieldImage),
            paint);
      }
      if (waypoint.prevControl != null) {
        canvas.drawLine(
            PathPainterUtil.pointToPixelOffset(
                waypoint.anchor, scale, fieldImage),
            PathPainterUtil.pointToPixelOffset(
                waypoint.prevControl!, scale, fieldImage),
            paint);
      }
    }

    if (waypointIdx == 0) {
      paint.color = Colors.green;
    } else if (waypointIdx == path.waypoints.length - 1) {
      paint.color = Colors.red;
    } else {
      paint.color = colorScheme.secondary;
    }

    if (waypointIdx == selectedWaypoint) {
      paint.color = Colors.orange;
    } else if (waypointIdx == hoveredWaypoint) {
      paint.color = Colors.deepPurpleAccent;
    }

    // draw anchor point
    paint.style = PaintingStyle.fill;
    canvas.drawCircle(
        PathPainterUtil.pointToPixelOffset(waypoint.anchor, scale, fieldImage),
        PathPainterUtil.uiPointSizeToPixels(25, scale, fieldImage),
        paint);
    paint.style = PaintingStyle.stroke;
    paint.color = colorScheme.surfaceContainer;
    canvas.drawCircle(
        PathPainterUtil.pointToPixelOffset(waypoint.anchor, scale, fieldImage),
        PathPainterUtil.uiPointSizeToPixels(25, scale, fieldImage),
        paint);

    if (!simple) {
      // draw control points
      if (waypoint.nextControl != null) {
        paint.style = PaintingStyle.fill;
        if (waypointIdx == selectedWaypoint) {
          paint.color = Colors.orange;
        } else if (waypointIdx == hoveredWaypoint) {
          paint.color = Colors.deepPurpleAccent;
        } else {
          paint.color = colorScheme.secondary;
        }

        canvas.drawCircle(
            PathPainterUtil.pointToPixelOffset(
                waypoint.nextControl!, scale, fieldImage),
            PathPainterUtil.uiPointSizeToPixels(20, scale, fieldImage),
            paint);
        paint.style = PaintingStyle.stroke;
        paint.color = colorScheme.surfaceContainer;
        canvas.drawCircle(
            PathPainterUtil.pointToPixelOffset(
                waypoint.nextControl!, scale, fieldImage),
            PathPainterUtil.uiPointSizeToPixels(20, scale, fieldImage),
            paint);
      }
      if (waypoint.prevControl != null) {
        paint.style = PaintingStyle.fill;
        if (waypointIdx == selectedWaypoint) {
          paint.color = Colors.orange;
        } else if (waypointIdx == hoveredWaypoint) {
          paint.color = Colors.deepPurpleAccent;
        } else {
          paint.color = colorScheme.secondary;
        }

        canvas.drawCircle(
            PathPainterUtil.pointToPixelOffset(
                waypoint.prevControl!, scale, fieldImage),
            PathPainterUtil.uiPointSizeToPixels(20, scale, fieldImage),
            paint);
        paint.style = PaintingStyle.stroke;
        paint.color = colorScheme.surfaceContainer;
        canvas.drawCircle(
            PathPainterUtil.pointToPixelOffset(
                waypoint.prevControl!, scale, fieldImage),
            PathPainterUtil.uiPointSizeToPixels(20, scale, fieldImage),
            paint);
      }
    }
  }

  void _paintGrid(Canvas canvas, Size size, bool showGrid) {
    if (!showGrid) return;

    final paint = Paint()
      ..color = colorScheme.secondary.withAlpha(50) // More transparent
      ..strokeWidth = 1;

    double gridSpacing = PathPainterUtil.metersToPixels(0.5, scale, fieldImage);

    for (double x = 0; x <= size.width; x += gridSpacing) {
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), paint);
    }

    for (double y = 0; y <= size.height; y += gridSpacing) {
      canvas.drawLine(Offset(0, y), Offset(size.width, y), paint);
    }
  }
}
