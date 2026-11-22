import 'dart:math';

import 'package:flutter/material.dart';
import 'package:pathplanner/path/obstacle_region.dart';
import 'package:pathplanner/path/pathplanner_path.dart';
import 'package:pathplanner/path/pickup_zone.dart';
import 'package:pathplanner/widgets/editor/tree_widgets/item_count.dart';
import 'package:pathplanner/widgets/editor/tree_widgets/tree_card_node.dart';
import 'package:pathplanner/widgets/number_text_field.dart';
import 'package:pathplanner/util/wpimath/geometry.dart';
import 'package:undo/undo.dart';

class ObstacleRegionsTree extends StatefulWidget {
  final PathPlannerPath path;
  final VoidCallback? onPathChanged;
  final VoidCallback? onPathChangedNoSim;
  final ValueChanged<int?>? onRegionHovered;
  final ValueChanged<int?>? onRegionSelected;
  final ValueChanged<int>? onEditDivert;
  final ChangeStack undoStack;

  const ObstacleRegionsTree({
    super.key,
    required this.path,
    this.onPathChanged,
    this.onPathChangedNoSim,
    this.onRegionHovered,
    this.onRegionSelected,
    this.onEditDivert,
    required this.undoStack,
  });

  @override
  State<ObstacleRegionsTree> createState() => _ObstacleRegionsTreeState();
}

class _ObstacleRegionsTreeState extends State<ObstacleRegionsTree> {
  List<ObstacleRegion> get regions => widget.path.obstacleRegions;

  final List<ExpansibleController> _controllers = [];
  final ExpansibleController _listController = ExpansibleController();

  /// The highest position a region can use, from the last waypoint
  double get _maxPos => max(1, widget.path.waypoints.length - 1).toDouble();

  @override
  void initState() {
    super.initState();
    _syncControllers();
  }

  @override
  void didUpdateWidget(covariant ObstacleRegionsTree oldWidget) {
    super.didUpdateWidget(oldWidget);
    _syncControllers();
  }

  void _syncControllers() {
    // Game piece regions always use a zone. Older files may not have one yet.
    for (final region in regions) {
      if (region.trigger == RegionTrigger.gamePiece &&
          region.pickupZone == null) {
        region.pickupZone = _defaultZone(region);
      }
    }
    while (_controllers.length < regions.length) {
      _controllers.add(ExpansibleController());
    }
    while (_controllers.length > regions.length) {
      _controllers.removeLast();
    }
  }

  /// A zone that surrounds the region's part of the path
  List<Translation2d> _defaultZone(ObstacleRegion region) {
    return PickupZone.defaultZone(widget.path, region);
  }

  /// After the region's range changes, refit the zone if a corner is now too far from it
  void _fitZoneToRange(ObstacleRegion region) {
    final zone = region.pickupZone;
    if (region.trigger != RegionTrigger.gamePiece || zone == null) return;
    for (final c in zone) {
      final clamped = PickupZone.clampCorner(widget.path, region, c);
      if (clamped.getDistance(c) > 1e-6) {
        region.pickupZone = _defaultZone(region);
        return;
      }
    }
  }

  /// Open a region's card and select it so it shows on the field
  void _openRegion(int index) {
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (!mounted) return;
      if (!_listController.isExpanded) _listController.expand();
      // The region's card only exists once the list is open, so wait one more frame
      WidgetsBinding.instance.addPostFrameCallback((_) {
        if (!mounted || index >= _controllers.length) return;
        if (!_controllers[index].isExpanded) _controllers[index].expand();
        widget.onRegionSelected?.call(index);
      });
      WidgetsBinding.instance.scheduleFrame();
    });
  }

  /// Make a change that can be undone. [change] edits the path's region list.
  void _edit(void Function() change) {
    final before = PathPlannerPath.cloneObstacleRegions(regions);
    widget.undoStack.add(Change(
      before,
      () {
        change();
        _syncControllers();
        setState(() {});
        widget.onPathChangedNoSim?.call();
      },
      (oldValue) {
        widget.path.obstacleRegions =
            PathPlannerPath.cloneObstacleRegions(oldValue);
        _syncControllers();
        setState(() {});
        widget.onRegionSelected?.call(null);
        widget.onPathChangedNoSim?.call();
      },
    ));
  }

  @override
  Widget build(BuildContext context) {
    return TreeCardNode(
      controller: _listController,
      title: const Text('Path Regions'),
      leading: const Icon(Icons.warning_amber_rounded),
      trailing: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          IconButton(
            icon: const Icon(Icons.add, size: 20),
            tooltip: 'Add obstacle region',
            onPressed: () {
              _edit(() {
                regions.add(ObstacleRegion(startPos: 0, endPos: _maxPos));
              });
              _openRegion(regions.length - 1);
            },
          ),
          const SizedBox(width: 8),
          ItemCount(count: regions.length),
        ],
      ),
      elevation: 1.0,
      children: [
        if (regions.isEmpty)
          const Center(
            child: Text(
                'Add a region to watch part of this path for a robot in the way or a game piece to pick up'),
          ),
        for (int i = 0; i < regions.length; i++) _buildRegionCard(i),
      ],
    );
  }

  Widget _buildRegionCard(int index) {
    final region = regions[index];

    return TreeCardNode(
      controller: _controllers[index],
      onHoverStart: () => widget.onRegionHovered?.call(index),
      onHoverEnd: () => widget.onRegionHovered?.call(null),
      onExpansionChanged: (expanded) {
        if (expanded ?? false) {
          widget.onRegionSelected?.call(index);
        } else {
          widget.onRegionSelected?.call(null);
        }
      },
      title: Text(
          'Region ${index + 1}: ${region.startPos.toStringAsFixed(2)} to ${region.endPos.toStringAsFixed(2)}'),
      children: [
        const Text('Part of the path to watch'),
        RangeSlider(
          values:
              RangeValues(region.startPos.toDouble(), region.endPos.toDouble()),
          min: 0,
          max: _maxPos,
          divisions: (_maxPos * 20).round(),
          labels: RangeLabels(region.startPos.toStringAsFixed(2),
              region.endPos.toStringAsFixed(2)),
          onChanged: (values) {
            setState(() {
              region.startPos = values.start;
              region.endPos = values.end;
            });
            widget.onPathChangedNoSim?.call();
          },
          onChangeEnd: (_) {
            setState(() => _fitZoneToRange(region));
            widget.onPathChanged?.call();
          },
        ),
        const SizedBox(height: 8),
        SegmentedButton<RegionTrigger>(
          segments: const [
            ButtonSegment(
                value: RegionTrigger.robot,
                icon: Icon(Icons.smart_toy_outlined),
                label: Text('Robot in the way')),
            ButtonSegment(
                value: RegionTrigger.gamePiece,
                icon: Icon(Icons.sports_baseball_outlined),
                label: Text('Game piece')),
          ],
          selected: {region.trigger},
          onSelectionChanged: (selection) {
            _edit(() {
              region.trigger = selection.first;
              if (region.trigger == RegionTrigger.gamePiece) {
                region.pickupZone ??= _defaultZone(region);
              }
            });
            widget.onRegionSelected?.call(index);
          },
        ),
        if (region.trigger == RegionTrigger.gamePiece) ...[
          const SizedBox(height: 8),
          const Text(
              'While the robot is in this part of the path, any game piece seen inside the orange zone is picked up. Pieces outside the zone are ignored. Drag the 4 orange dots on the field to stretch or shrink the zone. It can reach about 3/4 of the region\'s length away from the path (1 to 4 m).'),
          const SizedBox(height: 4),
          Align(
            alignment: Alignment.centerLeft,
            child: TextButton.icon(
              icon: const Icon(Icons.center_focus_strong_rounded, size: 18),
              label: const Text('Reset zone around this region'),
              onPressed: () {
                _edit(() => region.pickupZone = _defaultZone(region));
                widget.onRegionSelected?.call(index);
              },
            ),
          ),
        ],
        if (region.trigger == RegionTrigger.robot) ...[
          const SizedBox(height: 8),
          Row(
            children: [
              const Text('Obstacles:'),
              const SizedBox(width: 12),
              DropdownButton<ObstacleTeam>(
                value: region.obstacleTeam,
                items: [
                  for (final team in ObstacleTeam.values)
                    DropdownMenuItem(value: team, child: Text(team.name)),
                ],
                onChanged: (value) {
                  if (value != null) {
                    _edit(() => region.obstacleTeam = value);
                  }
                },
              ),
            ],
          ),
          const SizedBox(height: 8),
          NumberTextField(
            initialValue: region.blockTimeoutSec ?? 0,
            label: 'Block timeout (s), 0 = divert right away',
            minValue: 0,
            precision: 2,
            onSubmitted: (value) {
              _edit(() {
                region.blockTimeoutSec =
                    (value == null || value <= 0) ? null : value.toDouble();
              });
            },
          ),
        ],
        const SizedBox(height: 8),
        if (region.trigger == RegionTrigger.gamePiece)
          const Text('After the pickup:'),
        SegmentedButton<RegionAction>(
          segments: [
            ButtonSegment(
                value: RegionAction.continuePath,
                label: Text(region.trigger == RegionTrigger.gamePiece
                    ? 'Rejoin path'
                    : 'Continue path')),
            const ButtonSegment(
                value: RegionAction.divert, label: Text('Divert')),
          ],
          selected: {region.action},
          onSelectionChanged: (selection) {
            _edit(() => region.action = selection.first);
          },
        ),
        if (region.action == RegionAction.divert) ...[
          const SizedBox(height: 8),
          OutlinedButton.icon(
            icon: const Icon(Icons.edit_road_rounded, size: 18),
            label: Text(region.divertPath == null
                ? 'Draw diverge path'
                : 'Edit diverge path'),
            onPressed: () => widget.onEditDivert?.call(index),
          ),
        ],
        const SizedBox(height: 8),
        Align(
          alignment: Alignment.centerRight,
          child: TextButton.icon(
            icon: const Icon(Icons.delete_outline_rounded, size: 18),
            label: const Text('Delete region'),
            onPressed: () => _edit(() => regions.removeAt(index)),
          ),
        ),
      ],
    );
  }
}
