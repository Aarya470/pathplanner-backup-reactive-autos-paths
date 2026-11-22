import 'dart:math';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:multi_split_view/multi_split_view.dart';
import 'package:pathplanner/path/constraints_zone.dart';
import 'package:pathplanner/path/event_marker.dart';
import 'package:pathplanner/path/path_constraints.dart';
import 'package:pathplanner/path/pathplanner_path.dart';
import 'package:pathplanner/path/pickup_zone.dart';
import 'package:pathplanner/path/pivot_route.dart';
import 'package:pathplanner/path/point_towards_zone.dart';
import 'package:pathplanner/path/rotation_target.dart';
import 'package:pathplanner/path/waypoint.dart';
import 'package:pathplanner/services/log.dart';
import 'package:pathplanner/services/pplib_telemetry.dart';
import 'package:pathplanner/trajectory/config.dart';
import 'package:pathplanner/trajectory/trajectory.dart';
import 'package:pathplanner/util/prefs.dart';
import 'package:pathplanner/util/wpimath/geometry.dart';
import 'package:pathplanner/widgets/dialogs/trajectory_render_dialog.dart';
import 'package:pathplanner/widgets/editor/path_painter.dart';
import 'package:pathplanner/widgets/editor/preview_seekbar.dart';
import 'package:pathplanner/widgets/editor/runtime_display.dart';
import 'package:pathplanner/widgets/editor/tree_widgets/path_tree.dart';
import 'package:pathplanner/widgets/editor/tree_widgets/waypoints_tree.dart';
import 'package:pathplanner/widgets/field_image.dart';
import 'package:pathplanner/util/path_painter_util.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:pathplanner/path/obstacle_region.dart';
import 'package:undo/undo.dart';

class SplitPathEditor extends StatefulWidget {
  final SharedPreferences prefs;
  final PathPlannerPath path;
  final FieldImage fieldImage;
  final ChangeStack undoStack;
  final PPLibTelemetry? telemetry;
  final bool hotReload;
  final bool simulate;
  final VoidCallback? onPathChanged;
  final PathPlannerPath? ghostPath;

  const SplitPathEditor({
    required this.prefs,
    required this.path,
    required this.fieldImage,
    required this.undoStack,
    this.telemetry,
    this.hotReload = false,
    this.simulate = false,
    this.onPathChanged,
    this.ghostPath,
    super.key,
  });

  @override
  State<SplitPathEditor> createState() => _SplitPathEditorState();
}

class _SplitPathEditorState extends State<SplitPathEditor>
    with SingleTickerProviderStateMixin {
  final MultiSplitViewController _controller = MultiSplitViewController();
  final WaypointsTreeController _waypointsTreeController =
      WaypointsTreeController();
  int? _hoveredWaypoint;
  int? _selectedWaypoint;
  int? _hoveredZone;
  int? _selectedZone;
  int? _hoveredRegion;
  int? _selectedRegion;
  PathPlannerPath? _divertTarget;
  ObstacleRegion? _divertRegion;
  String? _selectedHandle;
  int _viewMode = 0; // 0 normal, 1 pivot (obstacle stays), 2 continue (obstacle clears)
  int _viewRegionIndex = -1;
  PathPlannerPath get _path => _divertTarget ?? widget.path;
  int? _hoveredRotTarget;
  int? _selectedRotTarget;
  int? _hoveredPointZone;
  int? _selectedPointZone;
  int? _hoveredMarker;
  int? _selectedMarker;
  late bool _treeOnRight;
  Waypoint? _draggedPoint;
  Waypoint? _dragOldValue;
  int? _draggedRotationIdx;
  Translation2d? _draggedRotationPos;
  Rotation2d? _dragRotationOldValue;
  int? _draggedZoneCorner;
  int? _draggedZoneRegion;
  List<Translation2d>? _zoneDragOldValue;
  PathPlannerTrajectory? _simTraj;
  bool _paused = false;
  late bool _holonomicMode;

  PathPlannerPath? _optimizedPath;

  late Size _robotSize;
  late Translation2d _bumperOffset;
  late AnimationController _previewController;

  List<Waypoint> get waypoints => _path.waypoints;

  RuntimeDisplay? _runtimeDisplay;

  @override
  void initState() {
    super.initState();

    _previewController = AnimationController(vsync: this);

    _holonomicMode =
        widget.prefs.getBool(PrefsKeys.holonomicMode) ?? Defaults.holonomicMode;

    _treeOnRight =
        widget.prefs.getBool(PrefsKeys.treeOnRight) ?? Defaults.treeOnRight;

    var width =
        widget.prefs.getDouble(PrefsKeys.robotWidth) ?? Defaults.robotWidth;
    var length =
        widget.prefs.getDouble(PrefsKeys.robotLength) ?? Defaults.robotLength;
    _robotSize = Size(width, length);
    _bumperOffset = Translation2d(
        widget.prefs.getDouble(PrefsKeys.bumperOffsetX) ??
            Defaults.bumperOffsetX,
        widget.prefs.getDouble(PrefsKeys.bumperOffsetY) ??
            Defaults.bumperOffsetY);

    double treeWeight = widget.prefs.getDouble(PrefsKeys.editorTreeWeight) ??
        Defaults.editorTreeWeight;
    _controller.areas = [
      Area(
        weight: _treeOnRight ? (1.0 - treeWeight) : treeWeight,
        minimalWeight: 0.4,
      ),
      Area(
        weight: _treeOnRight ? treeWeight : (1.0 - treeWeight),
        minimalWeight: 0.4,
      ),
    ];

    WidgetsBinding.instance.addPostFrameCallback((_) => _simulatePath());
  }

  @override
  void dispose() {
    _previewController.dispose();
    super.dispose();
  }

  /// Which handle of the diverge path is under the tap: the green start, the red end, or null
  String? _handleAt(Offset local) {
    if (_divertTarget == null || _path.waypoints.length < 2) return null;
    final Translation2d tap =
        Translation2d(_xPixelsToMeters(local.dx), _yPixelsToMeters(local.dy));
    if (_path.waypoints.last.anchor.getDistance(tap).toDouble() <= 0.4) return 'end';
    return null;
  }

  /// Move a waypoint's anchor and its controls together
  void _moveWaypoint(Waypoint w, Translation2d to) {
    final Translation2d delta = to - w.anchor;
    w.anchor = to;
    if (w.prevControl != null) w.prevControl = w.prevControl! + delta;
    if (w.nextControl != null) w.nextControl = w.nextControl! + delta;
  }

  /// Rotation the main path has at a position: its nearest rotation target, or the direction of travel
  Rotation2d _mainRotationAtPos(num pos) {
    final PathPlannerPath main = widget.path;
    if (main.rotationTargets.isNotEmpty) {
      RotationTarget nearest = main.rotationTargets.first;
      for (final t in main.rotationTargets) {
        if ((t.waypointRelativePos - pos).abs() < (nearest.waypointRelativePos - pos).abs()) {
          nearest = t;
        }
      }
      return nearest.rotation;
    }
    final Translation2d a = main.samplePath(pos);
    final Translation2d b = main.samplePath(pos + 0.05);
    final Translation2d d = b - a;
    if (d.x == 0 && d.y == 0) return const Rotation2d();
    return Rotation2d.fromDegrees(atan2(d.y.toDouble(), d.x.toDouble()) * 180 / pi);
  }

  /// Put the green start at the end of the region's yellow line, with the main path's rotation.
  /// The start cannot be moved anywhere else.
  void _syncDivertStart() {
    final PathPlannerPath? divert = _divertTarget;
    final ObstacleRegion? region = _divertRegion;
    if (divert == null || region == null) return;

    final Translation2d at = widget.path.samplePath(region.endPos);
    if (divert.waypoints.isEmpty) {
      divert.addWaypoint(at);
    } else {
      _moveWaypoint(divert.waypoints.first, at);
    }
    divert.rotationTargets.removeWhere((t) => t.waypointRelativePos == 0);
    divert.rotationTargets.add(RotationTarget(0, _mainRotationAtPos(region.endPos)));
    divert.generateAndSavePath();
  }

  /// Move the red end anywhere on the field
  void _moveDivertEnd(Offset local) {
    final Translation2d to =
        Translation2d(_xPixelsToMeters(local.dx), _yPixelsToMeters(local.dy));
    final before = PathPlannerPath.cloneWaypoints(_path.waypoints);
    widget.undoStack.add(Change(
      before,
      () {
        setState(() {
          _selectedHandle = null;
          _moveWaypoint(_path.waypoints.last, to);
          _generateAndSave();
        });
        widget.onPathChanged?.call();
      },
      (oldValue) {
        setState(() {
          _path.waypoints = PathPlannerPath.cloneWaypoints(oldValue);
          _generateAndSave();
        });
        widget.onPathChanged?.call();
      },
    ));
  }

  /// Regenerate the active path, then save the main path so pivots are written to its file
  void _generateAndSave() {
    _path.generateAndSavePath();
    widget.path.saveFile();
    _simulatePath();
  }

  /// True if the pivot shown in the selector has a timeout (so the Continue view exists)
  bool _selectedPivotHasTimeout() {
    if (_viewRegionIndex < 0 || _viewRegionIndex >= widget.path.obstacleRegions.length) {
      return false;
    }
    return widget.path.obstacleRegions[_viewRegionIndex].blockTimeoutSec != null;
  }

  /// Indexes of game piece regions with a zone that can be dragged, the selected one first.
  /// Only on the main path in the normal view.
  List<int> _draggableZoneRegions() {
    if (_divertTarget != null || _viewMode != 0) return const [];
    final regions = widget.path.obstacleRegions;
    final List<int> out = [];
    for (int i = 0; i < regions.length; i++) {
      final zone = regions[i].pickupZone;
      if (regions[i].trigger == RegionTrigger.gamePiece &&
          zone != null &&
          zone.length >= 3) {
        if (i == _selectedRegion) {
          out.insert(0, i);
        } else {
          out.add(i);
        }
      }
    }
    return out;
  }

  /// Save a finished zone corner drag so it can be undone
  void _finishZoneDrag() {
    final int regionIdx = _draggedZoneRegion!;
    final List<Translation2d> before = _zoneDragOldValue!;
    final List<Translation2d> after =
        List.of(widget.path.obstacleRegions[regionIdx].pickupZone!);
    _draggedZoneCorner = null;
    _draggedZoneRegion = null;
    _zoneDragOldValue = null;

    void setZone(List<Translation2d> zone) {
      if (regionIdx >= widget.path.obstacleRegions.length) return;
      setState(() {
        widget.path.obstacleRegions[regionIdx].pickupZone = List.of(zone);
        widget.path.saveFile();
      });
      if (widget.hotReload) {
        widget.telemetry?.hotReloadPath(widget.path);
      }
    }

    widget.undoStack.add(Change(
      before,
      () => setZone(after),
      (oldValue) => setZone(oldValue),
    ));
  }

  /// The main path up to the exact end of the region, then the pivot path
  PathPlannerPath _pivotRoutePath(ObstacleRegion region) =>
      PivotRoute.build(widget.path, region);

  /// Number of main path waypoints kept in a pivot route, minus one
  int _pivotSplitIndex(ObstacleRegion region) {
    final int n = widget.path.waypoints.length;
    return min(region.endPos.floor(), n - 2).clamp(0, n - 2);
  }

  /// The pivot route shown in Pivot view, or null when not in Pivot view
  PathPlannerPath? _shownPivotRoute() {
    if (_divertTarget != null || _viewMode != 1 || _viewRegionIndex < 0) {
      return null;
    }
    final ObstacleRegion region =
        widget.path.obstacleRegions[_viewRegionIndex];
    if (region.divertPath == null || region.divertPath!.waypoints.isEmpty) {
      return null;
    }
    return _pivotRoutePath(region);
  }

  /// The region whose timeout applies to the simulated route, or null
  ObstacleRegion? _waitRegion() {
    // Only the Continue view waits. The pivot route goes straight on.
    if (_divertTarget != null || _viewMode != 2 || _viewRegionIndex < 0) return null;
    return widget.path.obstacleRegions[_viewRegionIndex];
  }

  /// Hold the robot still at the end of the region for the timeout
  void _addWaitAtRegion(PathPlannerTrajectory traj) {
    final ObstacleRegion? region = _waitRegion();
    final double? timeout = region?.blockTimeoutSec?.toDouble();
    if (region == null || timeout == null || timeout <= 0) return;
    insertWaitAt(traj.states, widget.path.samplePath(region.endPos), timeout);
  }

  /// The path the moving robot should drive for the selected view
  PathPlannerPath _simulationPath() {
    if (_divertTarget != null && _divertRegion != null) {
      if (_divertTarget!.waypoints.isNotEmpty) return _pivotRoutePath(_divertRegion!);
      return widget.path;
    }
    if (_divertTarget == null && _viewMode == 1 && _viewRegionIndex >= 0) {
      final ObstacleRegion region = widget.path.obstacleRegions[_viewRegionIndex];
      if (region.divertPath != null && region.divertPath!.waypoints.isNotEmpty) {
        return _pivotRoutePath(region);
      }
    }
    return widget.path;
  }

  /// Go back to a valid view after regions change: a deleted region, a region no longer set to
  /// divert, or a timeout removed while in Continue view
  void _checkViewMode() {
    if (_viewMode == 0) return;
    final pivots = _pivotIndices();
    if (pivots.isEmpty) {
      _viewMode = 0;
      _viewRegionIndex = -1;
      return;
    }
    if (!pivots.contains(_viewRegionIndex)) _viewRegionIndex = pivots.first;
    if (_viewMode == 2 && !_selectedPivotHasTimeout()) _viewMode = 0;
  }

  /// Indices of the regions that have a diverge path, in order
  List<int> _pivotIndices() {
    final indices = <int>[];
    for (int i = 0; i < widget.path.obstacleRegions.length; i++) {
      if (widget.path.obstacleRegions[i].action == RegionAction.divert) indices.add(i);
    }
    return indices;
  }

  /// Switch the editor to a region's diverge path, on this same screen
  void _closeDivertEditor() {
    setState(() {
      _divertTarget = null;
      _divertRegion = null;
      _selectedHandle = null;
      _selectedWaypoint = null;
    });
    _simulatePath();
    widget.onPathChanged?.call();
  }

  /// Open the diverge path for a region on this same path. Creates it the first time.
  void _openDivertEditor(int index) {
    final region = _path.obstacleRegions[index];
    if (region.divertPath == null) {
      final diverge = PathPlannerPath.defaultPath(
        pathDir: _path.pathDir,
        fs: _path.fs,
        name: '${_path.name} diverge ${index + 1}',
      );
      diverge.isNested = true;
      region.divertPath = diverge;
      widget.onPathChanged?.call();
    }

    setState(() {
      _divertTarget = region.divertPath;
      _divertRegion = region;
      _selectedRegion = index;
      _selectedWaypoint = null;
    });
    _syncDivertStart();
    _simulatePath();
    widget.onPathChanged?.call();
  }

  @override
  Widget build(BuildContext context) {
    _checkViewMode();
    ColorScheme colorScheme = Theme.of(context).colorScheme;

    return Stack(
      children: [
        Center(
          child: InteractiveViewer(
            maxScale: 10.0,
            child: GestureDetector(
              onTapDown: (details) {
                FocusScopeNode currentScope = FocusScope.of(context);
                if (!currentScope.hasPrimaryFocus && currentScope.hasFocus) {
                  FocusManager.instance.primaryFocus!.unfocus();
                }
                final String? tapHandle = _handleAt(details.localPosition);
                if (tapHandle != null) {
                  setState(() => _selectedHandle = tapHandle);
                  return;
                }
                for (int i = waypoints.length - 1; i >= 0; i--) {
                  Waypoint w = waypoints[i];
                  if (w.isPointInAnchor(
                          _xPixelsToMeters(details.localPosition.dx),
                          _yPixelsToMeters(details.localPosition.dy),
                          _pixelsToMeters(PathPainterUtil.uiPointSizeToPixels(
                              25, PathPainter.scale, widget.fieldImage))) ||
                      w.isPointInNextControl(
                          _xPixelsToMeters(details.localPosition.dx),
                          _yPixelsToMeters(details.localPosition.dy),
                          _pixelsToMeters(PathPainterUtil.uiPointSizeToPixels(
                              20, PathPainter.scale, widget.fieldImage))) ||
                      w.isPointInPrevControl(
                          _xPixelsToMeters(details.localPosition.dx),
                          _yPixelsToMeters(details.localPosition.dy),
                          _pixelsToMeters(PathPainterUtil.uiPointSizeToPixels(
                              20, PathPainter.scale, widget.fieldImage)))) {
                    _setSelectedWaypoint(i);
                    return;
                  }
                }
                _setSelectedWaypoint(null);
              },
              onDoubleTapDown: (details) {
                final String? handle = _handleAt(details.localPosition);
                if (_selectedHandle == 'end' && _divertTarget != null) {
                  _moveDivertEnd(details.localPosition);
                  return;
                }
                if (handle != null) {
                  setState(() => _selectedHandle = handle);
                  return;
                }
                widget.undoStack.add(Change(
                  PathPlannerPath.cloneWaypoints(waypoints),
                  () {
                    setState(() {
                      _path.addWaypoint(Translation2d(
                          _xPixelsToMeters(details.localPosition.dx),
                          _yPixelsToMeters(details.localPosition.dy)));
                      _generateAndSave();
                    });
                    _simulatePath();
                  },
                  (oldValue) {
                    setState(() {
                      _path.waypoints =
                          PathPlannerPath.cloneWaypoints(oldValue);
                      _setSelectedWaypoint(null);
                      _generateAndSave();
                      _simulatePath();
                    });
                  },
                ));
              },
              onPanStart: (details) {
                double xPos = _xPixelsToMeters(details.localPosition.dx);
                double yPos = _yPixelsToMeters(details.localPosition.dy);

                // A corner of a game piece region's pickup zone, the selected region first
                final num grab = _pixelsToMeters(
                    PathPainterUtil.uiPointSizeToPixels(PathPainter.zoneCornerSize,
                        PathPainter.scale, widget.fieldImage));
                final Translation2d pointer = Translation2d(xPos, yPos);
                for (final int r in _draggableZoneRegions()) {
                  final zone = widget.path.obstacleRegions[r].pickupZone!;
                  for (int c = 0; c < zone.length; c++) {
                    if (zone[c].getDistance(pointer) <= grab) {
                      setState(() {
                        _draggedZoneRegion = r;
                        _draggedZoneCorner = c;
                        _zoneDragOldValue = List.of(zone);
                        _selectedRegion = r;
                      });
                      return;
                    }
                  }
                }

                for (int i = waypoints.length - 1; i >= 0; i--) {
                  Waypoint w = waypoints[i];
                  if (_divertTarget != null && i == 0) continue;
                  if (w.startDragging(
                      xPos,
                      yPos,
                      _pixelsToMeters(PathPainterUtil.uiPointSizeToPixels(
                          25, PathPainter.scale, widget.fieldImage)),
                      _pixelsToMeters(PathPainterUtil.uiPointSizeToPixels(
                          20, PathPainter.scale, widget.fieldImage)))) {
                    _draggedPoint = w;
                    _dragOldValue = w.clone();
                    break;
                  }
                }

                // Not dragging any waypoints, check rotations
                num dotRadius = _pixelsToMeters(
                    PathPainterUtil.uiPointSizeToPixels(
                        15, PathPainter.scale, widget.fieldImage));
                for (int i = 0; i < _path.pathPoints.length; i++) {
                  Rotation2d rotation;
                  Translation2d pos;
                  if (i == 0) {
                    rotation = _path.idealStartingState.rotation;
                    pos = _path.pathPoints.first.position;
                  } else if (i == _path.pathPoints.length - 1) {
                    rotation = _path.goalEndState.rotation;
                    pos = _path.pathPoints.last.position;
                  } else if (_path.pathPoints[i].rotationTarget != null) {
                    rotation =
                        _path.pathPoints[i].rotationTarget!.rotation;
                    pos = _path.pathPoints[i].position;
                  } else {
                    continue;
                  }

                  num dotX = pos.x +
                      (((_robotSize.height / 2) + _bumperOffset.x) *
                          rotation.cosine);
                  num dotY = pos.y +
                      (((_robotSize.height / 2) + _bumperOffset.x) *
                          rotation.sine);
                  if (pow(xPos - dotX, 2) + pow(yPos - dotY, 2) <
                      pow(dotRadius, 2)) {
                    if (i == 0) {
                      _draggedRotationIdx = -2;
                    } else if (i == _path.pathPoints.length - 2) {
                      _draggedRotationIdx = -1;
                    } else {
                      _draggedRotationIdx = _path.rotationTargets
                          .indexOf(_path.pathPoints[i].rotationTarget!);
                    }
                    _draggedRotationPos = pos;
                    _dragRotationOldValue = rotation;
                    return;
                  }
                }
              },
              onPanUpdate: (details) {
                if (_draggedZoneCorner != null && _draggedZoneRegion != null) {
                  final regions = widget.path.obstacleRegions;
                  if (_draggedZoneRegion! < regions.length) {
                    final region = regions[_draggedZoneRegion!];
                    final zone = region.pickupZone;
                    if (zone != null && _draggedZoneCorner! < zone.length) {
                      final moved = PickupZone.clampCorner(
                          widget.path,
                          region,
                          Translation2d(
                              _xPixelsToMeters(details.localPosition.dx),
                              _yPixelsToMeters(details.localPosition.dy)));
                      final candidate = List.of(zone);
                      candidate[_draggedZoneCorner!] = moved;
                      // Don't let the zone fold over itself
                      if (PickupZone.isSimple(candidate) ||
                          !PickupZone.isSimple(zone)) {
                        setState(() => zone[_draggedZoneCorner!] = moved);
                      }
                    }
                  }
                  return;
                }
                if (_draggedPoint != null) {
                  num targetX = _xPixelsToMeters(min(
                      88 +
                          (widget.fieldImage.defaultSize.width *
                              PathPainter.scale),
                      max(8, details.localPosition.dx)));
                  num targetY = _yPixelsToMeters(min(
                      88 +
                          (widget.fieldImage.defaultSize.height *
                              PathPainter.scale),
                      max(8, details.localPosition.dy)));

                  bool snapSetting =
                      widget.prefs.getBool(PrefsKeys.snapToGuidelines) ??
                          Defaults.snapToGuidelines;
                  bool ctrlHeld = HardwareKeyboard.instance.logicalKeysPressed
                          .contains(LogicalKeyboardKey.controlLeft) ||
                      HardwareKeyboard.instance.logicalKeysPressed
                          .contains(LogicalKeyboardKey.controlRight);

                  bool shouldSnap = snapSetting ^ ctrlHeld;

                  if (shouldSnap && _draggedPoint!.isAnchorDragging) {
                    num? closestX;
                    num? closestY;

                    for (Waypoint w in waypoints) {
                      if (w != _draggedPoint) {
                        if (closestX == null ||
                            (targetX - w.anchor.x).abs() <
                                (targetX - closestX).abs()) {
                          closestX = w.anchor.x;
                        }

                        if (closestY == null ||
                            (targetY - w.anchor.y).abs() <
                                (targetY - closestY).abs()) {
                          closestY = w.anchor.y;
                        }
                      }
                    }

                    if (closestX != null && (targetX - closestX).abs() < 0.1) {
                      targetX = closestX;
                    }
                    if (closestY != null && (targetY - closestY).abs() < 0.1) {
                      targetY = closestY;
                    }
                  }

                  setState(() {
                    _draggedPoint!.dragUpdate(targetX, targetY);
                    _path.generatePathPoints();
                  });
                } else if (_draggedRotationIdx != null) {
                  Translation2d pos;
                  if (_draggedRotationIdx == -2) {
                    pos = _path.waypoints.first.anchor;
                  } else if (_draggedRotationIdx == -1) {
                    pos = _path.waypoints.last.anchor;
                  } else {
                    pos = _draggedRotationPos!;
                  }

                  double x = _xPixelsToMeters(details.localPosition.dx);
                  double y = _yPixelsToMeters(details.localPosition.dy);

                  setState(() {
                    if (_draggedRotationIdx == -2) {
                      _path.idealStartingState.rotation =
                          Rotation2d.fromComponents(x - pos.x, y - pos.y);
                    } else if (_draggedRotationIdx == -1) {
                      _path.goalEndState.rotation =
                          Rotation2d.fromComponents(x - pos.x, y - pos.y);
                    } else {
                      _path.rotationTargets[_draggedRotationIdx!]
                              .rotation =
                          Rotation2d.fromComponents(x - pos.x, y - pos.y);
                    }
                  });
                }
              },
              onPanEnd: (details) {
                if (_draggedZoneCorner != null) {
                  _finishZoneDrag();
                  return;
                }
                if (_draggedPoint != null) {
                  _draggedPoint!.stopDragging();
                  int index = waypoints.indexOf(_draggedPoint!);
                  Waypoint dragEnd = _draggedPoint!.clone();
                  widget.undoStack.add(Change(
                    _dragOldValue,
                    () {
                      setState(() {
                        if (waypoints[index] != _draggedPoint) {
                          waypoints[index] = dragEnd.clone();
                        }
                        _generateAndSave();
                        _simulatePath();
                        widget.onPathChanged?.call();
                      });
                      if (widget.hotReload) {
                        widget.telemetry?.hotReloadPath(_path);
                      }
                    },
                    (oldValue) {
                      setState(() {
                        waypoints[index] = oldValue!.clone();
                        _generateAndSave();
                        _simulatePath();
                        widget.onPathChanged?.call();
                      });
                      if (widget.hotReload) {
                        widget.telemetry?.hotReloadPath(_path);
                      }
                    },
                  ));
                  _draggedPoint = null;
                } else if (_draggedRotationIdx != null) {
                  if (_draggedRotationIdx == -2) {
                    final endRotation = _path.idealStartingState.rotation;
                    widget.undoStack.add(Change(
                      _dragRotationOldValue,
                      () {
                        setState(() {
                          _path.idealStartingState.rotation = endRotation;
                          _generateAndSave();
                          _simulatePath();
                          widget.onPathChanged?.call();
                        });
                      },
                      (oldValue) {
                        setState(() {
                          _path.idealStartingState.rotation = oldValue!;
                          _generateAndSave();
                          _simulatePath();
                          widget.onPathChanged?.call();
                        });
                      },
                    ));
                  } else if (_draggedRotationIdx == -1) {
                    final endRotation = _path.goalEndState.rotation;
                    widget.undoStack.add(Change(
                      _dragRotationOldValue,
                      () {
                        setState(() {
                          _path.goalEndState.rotation = endRotation;
                          _generateAndSave();
                          _simulatePath();
                          widget.onPathChanged?.call();
                        });
                      },
                      (oldValue) {
                        setState(() {
                          _path.goalEndState.rotation = oldValue!;
                          _generateAndSave();
                          _simulatePath();
                          widget.onPathChanged?.call();
                        });
                      },
                    ));
                  } else {
                    int rotationIdx = _draggedRotationIdx!;
                    final endRotation =
                        _path.rotationTargets[rotationIdx].rotation;
                    widget.undoStack.add(Change(
                      _dragRotationOldValue,
                      () {
                        setState(() {
                          _path.rotationTargets[rotationIdx].rotation =
                              endRotation;
                          _generateAndSave();
                          _simulatePath();
                          widget.onPathChanged?.call();
                        });
                      },
                      (oldValue) {
                        setState(() {
                          _path.rotationTargets[rotationIdx].rotation =
                              oldValue!;
                          _generateAndSave();
                          _simulatePath();
                          widget.onPathChanged?.call();
                        });
                      },
                    ));
                  }
                  _draggedRotationIdx = null;
                  _draggedRotationPos = null;
                }
              },
              child: Padding(
                padding: const EdgeInsets.all(48),
                child: Stack(
                  children: [
                    widget.fieldImage.getWidget(),
                    Positioned.fill(
                      child: CustomPaint(
                        painter: PathPainter(
                          colorScheme: colorScheme,
                          paths: [_path],
                          simple: false,
                          fieldImage: widget.fieldImage,
                          hoveredWaypoint: _hoveredWaypoint,
                          selectedWaypoint: _selectedWaypoint,
                          hoveredZone: _hoveredZone,
                          selectedZone: _selectedZone,
                          hoveredRegion: _hoveredRegion,
                          selectedRegion: _selectedRegion,
                          ghostPath: _divertTarget != null ? widget.path : null,
                          highlightRegion: _divertTarget != null ? _divertRegion : null,
                          wholePivot: (_divertTarget == null && _viewMode == 1 && _viewRegionIndex >= 0)
                              ? widget.path.obstacleRegions[_viewRegionIndex]
                              : null,
                          continueRegion: (_divertTarget == null && _viewMode == 2 && _viewRegionIndex >= 0)
                              ? widget.path.obstacleRegions[_viewRegionIndex]
                              : null,
                          pivotRoute: _shownPivotRoute(),
                          showZoneHandles: _divertTarget == null && _viewMode == 0,
                          pivotSplitIndex: (_divertTarget == null && _viewMode == 1 && _viewRegionIndex >= 0)
                              ? _pivotSplitIndex(widget.path.obstacleRegions[_viewRegionIndex])
                              : -1,
                          hoveredPointZone: _hoveredPointZone,
                          selectedPointZone: _selectedPointZone,
                          hoveredRotTarget: _hoveredRotTarget,
                          selectedRotTarget: _selectedRotTarget,
                          hoveredMarker: _hoveredMarker,
                          selectedMarker: _selectedMarker,
                          simulatedPath: _simTraj,
                          animation: _previewController.view,
                          prefs: widget.prefs,
                          optimizedPath: _optimizedPath,
                        ),
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ),
        ),
        MultiSplitViewTheme(
          data: MultiSplitViewThemeData(
            dividerPainter: DividerPainters.grooved1(
              color: colorScheme.surfaceContainerHighest,
              highlightedColor: colorScheme.primary,
            ),
          ),
          child: MultiSplitView(
            axis: Axis.horizontal,
            controller: _controller,
            onWeightChange: () {
              double? newWeight = _treeOnRight
                  ? _controller.areas[1].weight
                  : _controller.areas[0].weight;
              widget.prefs
                  .setDouble(PrefsKeys.editorTreeWeight, newWeight ?? 0.5);
            },
            children: [
              if (_treeOnRight)
                PreviewSeekbar(
                  previewController: _previewController,
                  onPauseStateChanged: (value) => _paused = value,
                  totalPathTime: _simTraj?.states.last.timeSeconds ?? 1.0,
                ),
              Card(
                margin: const EdgeInsets.all(0),
                elevation: 4.0,
                color: colorScheme.surface,
                surfaceTintColor: colorScheme.surfaceTint,
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.only(
                    topLeft:
                        _treeOnRight ? const Radius.circular(12) : Radius.zero,
                    topRight:
                        _treeOnRight ? Radius.zero : const Radius.circular(12),
                    bottomLeft:
                        _treeOnRight ? const Radius.circular(12) : Radius.zero,
                    bottomRight:
                        _treeOnRight ? Radius.zero : const Radius.circular(12),
                  ),
                ),
                child: Padding(
                  padding: const EdgeInsets.all(8.0),
                  child: PathTree(
                    path: _path,
                    pathRuntime: _simTraj?.getTotalTimeSeconds(),
                    runtimeDisplay: _runtimeDisplay,
                    initiallySelectedWaypoint: _selectedWaypoint,
                    initiallySelectedZone: _selectedZone,
                    onExitDivert: _closeDivertEditor,
                    onRegionHovered: (value) {
                      setState(() {
                        _hoveredRegion = value;
                      });
                    },
                    onRegionSelected: (value) {
                      setState(() {
                        _selectedRegion = value;
                      });
                    },
                    onEditDivert: _openDivertEditor,
                    initiallySelectedRotTarget: _selectedRotTarget,
                    initiallySelectedPointZone: _selectedPointZone,
                    initiallySelectedMarker: _selectedMarker,
                    waypointsTreeController: _waypointsTreeController,
                    undoStack: widget.undoStack,
                    holonomicMode: _holonomicMode,
                    defaultConstraints: _getDefaultConstraints(),
                    prefs: widget.prefs,
                    fieldSizeMeters: widget.fieldImage.getFieldSizeMeters(),
                    onRenderPath: () {
                      if (_simTraj != null) {
                        showDialog(
                            context: context,
                            builder: (context) {
                              return TrajectoryRenderDialog(
                                fieldImage: widget.fieldImage,
                                prefs: widget.prefs,
                                trajectory: _simTraj!,
                              );
                            });
                      }
                    },
                    onPathChanged: () {
                      setState(() {
                        _generateAndSave();
                        _simulatePath();
                      });

                      if (widget.hotReload) {
                        widget.telemetry?.hotReloadPath(_path);
                      }

                      widget.onPathChanged?.call();
                    },
                    onPathChangedNoSim: () {
                      setState(() {
                        _generateAndSave();
                      });

                      if (widget.hotReload) {
                        widget.telemetry?.hotReloadPath(_path);
                      }

                      widget.onPathChanged?.call();
                    },
                    onWaypointDeleted: (waypointIdx) {
                      widget.undoStack.add(Change(
                        [
                          PathPlannerPath.cloneWaypoints(_path.waypoints),
                          PathPlannerPath.cloneConstraintZones(
                              _path.constraintZones),
                          PathPlannerPath.cloneEventMarkers(
                              _path.eventMarkers),
                          PathPlannerPath.cloneRotationTargets(
                              _path.rotationTargets),
                          PathPlannerPath.clonePointTowardsZones(
                              _path.pointTowardsZones),
                        ],
                        () {
                          setState(() {
                            _selectedWaypoint = null;
                            _hoveredWaypoint = null;
                            _waypointsTreeController.setSelectedWaypoint(null);

                            Waypoint w =
                                _path.waypoints.removeAt(waypointIdx);

                            if (w.isEndPoint) {
                              waypoints[_path.waypoints.length - 1]
                                  .nextControl = null;
                            } else if (w.isStartPoint) {
                              waypoints[0].prevControl = null;
                            }

                            for (ConstraintsZone zone
                                in _path.constraintZones) {
                              zone.minWaypointRelativePos =
                                  _adjustDeletedWaypointRelativePos(
                                      zone.minWaypointRelativePos, waypointIdx);
                              zone.maxWaypointRelativePos =
                                  _adjustDeletedWaypointRelativePos(
                                      zone.maxWaypointRelativePos, waypointIdx);
                            }

                            for (PointTowardsZone zone
                                in _path.pointTowardsZones) {
                              zone.minWaypointRelativePos =
                                  _adjustDeletedWaypointRelativePos(
                                      zone.minWaypointRelativePos, waypointIdx);
                              zone.maxWaypointRelativePos =
                                  _adjustDeletedWaypointRelativePos(
                                      zone.maxWaypointRelativePos, waypointIdx);
                            }

                            for (EventMarker m in _path.eventMarkers) {
                              m.waypointRelativePos =
                                  _adjustDeletedWaypointRelativePos(
                                      m.waypointRelativePos, waypointIdx);
                            }

                            for (RotationTarget t
                                in _path.rotationTargets) {
                              t.waypointRelativePos =
                                  _adjustDeletedWaypointRelativePos(
                                      t.waypointRelativePos, waypointIdx);
                            }

                            _generateAndSave();
                            _simulatePath();
                          });
                        },
                        (oldValue) {
                          setState(() {
                            _selectedWaypoint = null;
                            _hoveredWaypoint = null;
                            _waypointsTreeController.setSelectedWaypoint(null);

                            _path.waypoints =
                                PathPlannerPath.cloneWaypoints(
                                    oldValue[0] as List<Waypoint>);
                            _path.constraintZones =
                                PathPlannerPath.cloneConstraintZones(
                                    oldValue[1] as List<ConstraintsZone>);
                            _path.eventMarkers =
                                PathPlannerPath.cloneEventMarkers(
                                    oldValue[2] as List<EventMarker>);
                            _path.rotationTargets =
                                PathPlannerPath.cloneRotationTargets(
                                    oldValue[3] as List<RotationTarget>);
                            _path.pointTowardsZones =
                                PathPlannerPath.clonePointTowardsZones(
                                    oldValue[4] as List<PointTowardsZone>);
                            _generateAndSave();
                            _simulatePath();
                          });
                        },
                      ));
                    },
                    onSideSwapped: () => setState(() {
                      _treeOnRight = !_treeOnRight;
                      widget.prefs.setBool(PrefsKeys.treeOnRight, _treeOnRight);
                      _controller.areas = _controller.areas.reversed.toList();
                    }),
                    onWaypointHovered: (value) {
                      setState(() {
                        _hoveredWaypoint = value;
                      });
                    },
                    onWaypointSelected: (value) {
                      setState(() {
                        _selectedWaypoint = value;
                      });
                    },
                    onZoneHovered: (value) {
                      setState(() {
                        _hoveredZone = value;
                      });
                    },
                    onZoneSelected: (value) {
                      setState(() {
                        _selectedZone = value;
                      });
                    },
                    onPointZoneHovered: (value) {
                      setState(() {
                        _hoveredPointZone = value;
                      });
                    },
                    onPointZoneSelected: (value) {
                      setState(() {
                        _selectedPointZone = value;
                      });
                    },
                    onRotTargetHovered: (value) {
                      setState(() {
                        _hoveredRotTarget = value;
                      });
                    },
                    onRotTargetSelected: (value) {
                      setState(() {
                        _selectedRotTarget = value;
                      });
                    },
                    onMarkerHovered: (value) {
                      setState(() {
                        _hoveredMarker = value;
                      });
                    },
                    onMarkerSelected: (value) {
                      setState(() {
                        _selectedMarker = value;
                      });
                    },
                    onOptimizationUpdate: (result) => setState(() {
                      _optimizedPath = result;
                    }),
                  ),
                ),
              ),
              if (!_treeOnRight)
                PreviewSeekbar(
                  previewController: _previewController,
                  onPauseStateChanged: (value) => _paused = value,
                  totalPathTime: _simTraj?.states.last.timeSeconds ?? 1.0,
                ),
            ],
          ),
        ),
        if (_divertTarget != null)
          Positioned(
            top: 8,
            right: 8,
            child: FilledButton(
              onPressed: _closeDivertEditor,
              child: const Text('Live v11'),
            ),
          ),
        if (_divertTarget == null && _pivotIndices().isNotEmpty)
          Positioned(
            bottom: 8,
            left: 0,
            right: 0,
            child: Center(
              child: Card(
                child: Padding(
                  padding: const EdgeInsets.all(6),
                  child: Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      SegmentedButton<int>(
                        segments: [
                          const ButtonSegment(value: 0, label: Text('Normal')),
                          const ButtonSegment(value: 1, label: Text('Pivot')),
                          if (_selectedPivotHasTimeout())
                            const ButtonSegment(value: 2, label: Text('Continue')),
                        ],
                        selected: {_viewMode},
                        onSelectionChanged: (choice) {
                          setState(() {
                            _viewMode = choice.first;
                            if (_viewMode != 0 && !_pivotIndices().contains(_viewRegionIndex)) {
                              _viewRegionIndex = _pivotIndices().first;
                            }
                          });
                          _simulatePath();
                        },
                      ),
                      if (_viewMode != 0 && _pivotIndices().length > 1) ...[
                        const SizedBox(width: 12),
                        DropdownButton<int>(
                          value: _viewRegionIndex,
                          items: [
                            for (int k = 0; k < _pivotIndices().length; k++)
                              DropdownMenuItem(
                                  value: _pivotIndices()[k], child: Text('Pivot ${k + 1}')),
                          ],
                          onChanged: (value) {
                            if (value != null) {
                              setState(() => _viewRegionIndex = value);
                              _simulatePath();
                            }
                          },
                        ),
                      ],
                    ],
                  ),
                ),
              ),
            ),
          ),
      ],
    );
  }

  // marked as async so it can be called from initState
  void _simulatePath() async {
    _checkViewMode();
    if (widget.simulate) {
      setState(() {
        _simTraj = PathPlannerTrajectory(
          path: _simulationPath(),
          robotConfig: RobotConfig.fromPrefs(widget.prefs),
        );
        if (_simTraj != null) _addWaitAtRegion(_simTraj!);
        if (!(_simTraj?.getTotalTimeSeconds().isFinite ?? false)) {
          _simTraj = null;
        }

        // Update the RuntimeDisplay widget
        _runtimeDisplay = RuntimeDisplay(
          currentRuntime: _simTraj?.states.last.timeSeconds,
          previousRuntime: _runtimeDisplay?.currentRuntime,
        );
      });

      if (!_paused) {
        _previewController.stop();
        _previewController.reset();
      }

      if (_simTraj != null) {
        try {
          if (!_paused) {
            _previewController.stop();
            _previewController.reset();
            _previewController.duration = Duration(
                milliseconds:
                    (_simTraj!.states.last.timeSeconds * 1000).toInt());
            _previewController.repeat();
          } else if (_previewController.duration != null) {
            double prevTime = _previewController.value *
                (_previewController.duration!.inMilliseconds / 1000.0);
            _previewController.duration = Duration(
                milliseconds:
                    (_simTraj!.states.last.timeSeconds * 1000).toInt());
            double newPos = prevTime / _simTraj!.states.last.timeSeconds;
            _previewController.forward(from: newPos);
            _previewController.stop();
          }
        } catch (_) {
          _showGenerationFailedError();
        }
      } else {
        // Trajectory failed to generate. Notify the user
        _showGenerationFailedError();
      }
    }
  }

  void _showGenerationFailedError() {
    Log.warning('Failed to generate trajectory for path: ${_path.name}');
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(
          'Failed to generate trajectory. This is likely due to bad control point placement. Please adjust your control points to avoid kinks in the path.',
          style:
              TextStyle(color: Theme.of(context).colorScheme.onErrorContainer),
        ),
        backgroundColor: Theme.of(context).colorScheme.errorContainer,
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(
          borderRadius: BorderRadius.circular(8),
        ),
        action: SnackBarAction(
          label: 'Dismiss',
          textColor: Theme.of(context).colorScheme.onErrorContainer,
          onPressed: () {
            ScaffoldMessenger.of(context).hideCurrentSnackBar();
          },
        ),
      ),
    );
  }

  num _adjustDeletedWaypointRelativePos(num pos, int deletedWaypointIdx) {
    if (pos >= deletedWaypointIdx + 1) {
      return pos - 1.0;
    } else if (pos >= deletedWaypointIdx) {
      int segment = pos.floor();
      double segmentPct = pos % 1.0;

      return max(
          (((segment - 0.5) + (segmentPct / 2.0)) * 20).round() / 20.0, 0.0);
    } else if (pos > deletedWaypointIdx - 1) {
      int segment = pos.floor();
      double segmentPct = pos % 1.0;

      return min(_path.waypoints.length - 1,
          ((segment + (0.5 * segmentPct)) * 20).round() / 20.0);
    }

    return pos;
  }

  void _setSelectedWaypoint(int? waypointIdx) {
    setState(() {
      _selectedWaypoint = waypointIdx;
    });

    _waypointsTreeController.setSelectedWaypoint(waypointIdx);
  }

  double _xPixelsToMeters(double pixels) {
    return (((pixels - 48) / PathPainter.scale) /
            widget.fieldImage.pixelsPerMeter) -
        widget.fieldImage.marginMeters;
  }

  double _yPixelsToMeters(double pixels) {
    return ((widget.fieldImage.defaultSize.height -
                ((pixels - 48) / PathPainter.scale)) /
            widget.fieldImage.pixelsPerMeter) -
        widget.fieldImage.marginMeters;
  }

  double _pixelsToMeters(double pixels) {
    return (pixels / PathPainter.scale) / widget.fieldImage.pixelsPerMeter;
  }

  PathConstraints _getDefaultConstraints() {
    return PathConstraints(
      maxVelocityMPS: widget.prefs.getDouble(PrefsKeys.defaultMaxVel) ??
          Defaults.defaultMaxVel,
      maxAccelerationMPSSq: widget.prefs.getDouble(PrefsKeys.defaultMaxAccel) ??
          Defaults.defaultMaxAccel,
      maxAngularVelocityDeg:
          widget.prefs.getDouble(PrefsKeys.defaultMaxAngVel) ??
              Defaults.defaultMaxAngVel,
      maxAngularAccelerationDeg:
          widget.prefs.getDouble(PrefsKeys.defaultMaxAngAccel) ??
              Defaults.defaultMaxAngAccel,
    );
  }
}
