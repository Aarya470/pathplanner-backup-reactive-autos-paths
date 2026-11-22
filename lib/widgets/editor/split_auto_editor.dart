import 'package:flutter/material.dart';
import 'package:multi_split_view/multi_split_view.dart';
import 'package:pathplanner/auto/pathplanner_auto.dart';
import 'package:pathplanner/path/choreo_path.dart';
import 'package:pathplanner/services/log.dart';
import 'package:pathplanner/trajectory/auto_simulator.dart';
import 'package:pathplanner/trajectory/config.dart';
import 'package:pathplanner/trajectory/trajectory.dart';
import 'package:pathplanner/path/obstacle_region.dart';
import 'package:pathplanner/path/pathplanner_path.dart';
import 'package:pathplanner/path/pivot_route.dart';
import 'package:pathplanner/util/prefs.dart';
import 'package:pathplanner/widgets/dialogs/trajectory_render_dialog.dart';
import 'package:pathplanner/widgets/editor/path_painter.dart';
import 'package:pathplanner/widgets/editor/preview_seekbar.dart';
import 'package:pathplanner/widgets/editor/tree_widgets/auto_tree.dart';
import 'package:pathplanner/widgets/field_image.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:undo/undo.dart';

class SplitAutoEditor extends StatefulWidget {
  final SharedPreferences prefs;
  final PathPlannerAuto auto;
  final List<PathPlannerPath> autoPaths;
  final List<ChoreoPath> autoChoreoPaths;
  final List<String> allPathNames;
  final VoidCallback? onAutoChanged;
  final FieldImage fieldImage;
  final ChangeStack undoStack;
  final Function(String?)? onEditPathPressed;

  const SplitAutoEditor({
    required this.prefs,
    required this.auto,
    required this.autoPaths,
    required this.autoChoreoPaths,
    required this.allPathNames,
    required this.fieldImage,
    required this.undoStack,
    this.onAutoChanged,
    this.onEditPathPressed,
    super.key,
  });

  @override
  State<SplitAutoEditor> createState() => _SplitAutoEditorState();
}

class _SplitAutoEditorState extends State<SplitAutoEditor>
    with SingleTickerProviderStateMixin {
  final MultiSplitViewController _controller = MultiSplitViewController();
  String? _hoveredPath;
  late bool _treeOnRight;
  PathPlannerTrajectory? _simTraj;
  bool _paused = false;

  /// 0 shows the normal paths, 1 the pivot routes, 2 the robot waiting then continuing
  int _viewMode = 0;

  /// The first region of a path that has a drawn diverge path
  static ObstacleRegion? _pivotRegion(PathPlannerPath path) {
    for (final r in path.obstacleRegions) {
      if (r.action == RegionAction.divert &&
          r.divertPath != null &&
          r.divertPath!.waypoints.isNotEmpty) {
        return r;
      }
    }
    return null;
  }

  /// The first region of a path where the robot waits for a blocker before going on
  static ObstacleRegion? _waitRegion(PathPlannerPath path) {
    for (final r in path.obstacleRegions) {
      if (r.trigger == RegionTrigger.robot && (r.blockTimeoutSec ?? 0) > 0) {
        return r;
      }
    }
    return null;
  }

  bool get _hasPivot =>
      !widget.auto.choreoAuto &&
      widget.autoPaths.any((p) => _pivotRegion(p) != null);

  bool get _hasContinue =>
      !widget.auto.choreoAuto &&
      widget.autoPaths.any((p) => _waitRegion(p) != null);

  /// The paths to draw and simulate for the chosen view. In Pivot view, the paths run up to the
  /// first one with a diverge path, which is replaced by its pivot route. The auto's later paths
  /// start where the main path ends, not where the diverge path ends, so they are left out.
  List<PathPlannerPath>? _pivotPaths;

  List<PathPlannerPath> _shownPaths() {
    if (_viewMode != 1) return widget.autoPaths;
    if (_pivotPaths != null) return _pivotPaths!;
    final List<PathPlannerPath> shown = [];
    for (final p in widget.autoPaths) {
      final r = _pivotRegion(p);
      if (r == null) {
        shown.add(p);
        continue;
      }
      final route = PivotRoute.build(p, r);
      // Keep the original name, so hovering the path in the list still highlights it
      route.name = p.name;
      shown.add(route);
      break;
    }
    return _pivotPaths = shown;
  }

  /// True if Pivot view leaves out paths after the pivot
  bool get _pivotCutsAuto =>
      _viewMode == 1 && _shownPaths().length < widget.autoPaths.length;

  /// In Continue view, hold the robot still at the end of each waiting region. Each wait is
  /// only looked for inside its own path's part of the trajectory.
  void _addWaits(PathPlannerTrajectory traj, List<int> pathStarts) {
    if (_viewMode != 2) return;
    final int originalLength = traj.states.length;
    int inserted = 0;
    for (int i = 0; i < widget.autoPaths.length && i < pathStarts.length; i++) {
      final p = widget.autoPaths[i];
      final r = _waitRegion(p);
      if (r == null) continue;
      final int start = pathStarts[i] + inserted;
      final int end =
          (i + 1 < pathStarts.length ? pathStarts[i + 1] : originalLength) +
              inserted;
      insertWaitAt(traj.states, p.samplePath(r.endPos), r.blockTimeoutSec!,
          startIndex: start, endIndex: end);
      inserted = traj.states.length - originalLength;
    }
  }

  /// Go back to the normal view if the chosen one no longer applies
  void _checkViewMode() {
    if (_viewMode == 1 && !_hasPivot || _viewMode == 2 && !_hasContinue) {
      _viewMode = 0;
      _pivotPaths = null;
    }
  }

  @override
  void didUpdateWidget(covariant SplitAutoEditor oldWidget) {
    super.didUpdateWidget(oldWidget);
    _pivotPaths = null;
    _checkViewMode();
  }

  Widget _buildViewSelector() {
    if (!_hasPivot && !_hasContinue) return const SizedBox.shrink();
    return Positioned(
      top: 8,
      left: _treeOnRight ? 8 : null,
      right: _treeOnRight ? null : 8,
      child: Card(
        child: Padding(
          padding: const EdgeInsets.all(6),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              SegmentedButton<int>(
                segments: [
                  const ButtonSegment(value: 0, label: Text('Normal')),
                  if (_hasPivot)
                    const ButtonSegment(value: 1, label: Text('Pivot')),
                  if (_hasContinue)
                    const ButtonSegment(value: 2, label: Text('Continue')),
                ],
                selected: {_viewMode},
                onSelectionChanged: (choice) {
                  setState(() {
                    _viewMode = choice.first;
                    _pivotPaths = null;
                  });
                  _simulateAuto();
                },
              ),
              if (_pivotCutsAuto)
                const Padding(
                  padding: EdgeInsets.only(top: 4, left: 4),
                  child: Text('Shown up to where the robot pivots',
                      style: TextStyle(fontSize: 12)),
                ),
            ],
          ),
        ),
      ),
    );
  }

  late AnimationController _previewController;

  @override
  void initState() {
    super.initState();

    _previewController = AnimationController(vsync: this);

    _treeOnRight =
        widget.prefs.getBool(PrefsKeys.treeOnRight) ?? Defaults.treeOnRight;

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

    WidgetsBinding.instance.addPostFrameCallback((_) => _simulateAuto());
  }

  @override
  void dispose() {
    _previewController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    ColorScheme colorScheme = Theme.of(context).colorScheme;
    _checkViewMode();

    return Stack(
      children: [
        Center(
          child: InteractiveViewer(
            maxScale: 10.0,
            child: Padding(
              padding: const EdgeInsets.all(48),
              child: Stack(
                children: [
                  widget.fieldImage.getWidget(),
                  Positioned.fill(
                    child: CustomPaint(
                        painter: PathPainter(
                            colorScheme: colorScheme,
                            paths: _shownPaths(),
                            choreoPaths: widget.autoChoreoPaths,
                            simple: true,
                            hideOtherPathsOnHover: widget.prefs
                                    .getBool(PrefsKeys.hidePathsOnHover) ??
                                Defaults.hidePathsOnHover,
                            hoveredPath: _hoveredPath,
                            fieldImage: widget.fieldImage,
                            simulatedPath: _simTraj,
                            animation: _previewController.view,
                            prefs: widget.prefs)),
                  ),
                ],
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
                  child: AutoTree(
                    auto: widget.auto,
                    autoRuntime: _simTraj?.states.last.timeSeconds,
                    allPathNames: widget.allPathNames,
                    onRenderAuto: () {
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
                    onPathHovered: (value) {
                      setState(() {
                        _hoveredPath = value;
                      });
                    },
                    onAutoChanged: () {
                      widget.onAutoChanged?.call();
                      // Delay this because it needs the parent widget to rebuild first
                      Future.delayed(const Duration(milliseconds: 100))
                          .then((_) {
                        _simulateAuto();
                      });
                    },
                    onSideSwapped: () => setState(() {
                      _treeOnRight = !_treeOnRight;
                      widget.prefs.setBool(PrefsKeys.treeOnRight, _treeOnRight);
                      _controller.areas = _controller.areas.reversed.toList();
                    }),
                    undoStack: widget.undoStack,
                    onEditPathPressed: widget.onEditPathPressed,
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
        _buildViewSelector(),
      ],
    );
  }

  // Marked as async so it can run from initState
  void _simulateAuto() async {
    _pivotPaths = null;
    if (widget.autoPaths.isEmpty && widget.autoChoreoPaths.isEmpty) {
      setState(() {
        _simTraj = null;
      });

      _previewController.stop();
      _previewController.reset();

      return;
    }

    PathPlannerTrajectory? simPath;

    if (widget.auto.choreoAuto) {
      List<TrajectoryState> allStates = [];
      num timeOffset = 0.0;

      for (ChoreoPath p in widget.autoChoreoPaths) {
        for (TrajectoryState s in p.trajectory.states) {
          allStates.add(s.copyWithTime(s.timeSeconds + timeOffset));
        }

        if (allStates.isNotEmpty) {
          timeOffset = allStates.last.timeSeconds;
        }
      }

      if (allStates.isNotEmpty) {
        simPath = PathPlannerTrajectory.fromStates(allStates);
      }
    } else {
      RobotConfig config = RobotConfig.fromPrefs(widget.prefs);

      try {
        final List<int> pathStarts = [];
        simPath = AutoSimulator.simulateAuto(
          _shownPaths(),
          config,
          pathStarts: pathStarts,
        );
        if (simPath != null) _addWaits(simPath, pathStarts);
        if (!(simPath?.getTotalTimeSeconds().isFinite ?? false)) {
          simPath = null;
        }
      } catch (err) {
        Log.error('Failed to simulate auto', err);
      }
    }

    if (simPath != null &&
        simPath.states.last.timeSeconds.isFinite &&
        !simPath.states.last.timeSeconds.isNaN) {
      setState(() {
        _simTraj = simPath;
      });

      try {
        if (!_paused) {
          _previewController.stop();
          _previewController.reset();
          _previewController.duration = Duration(
              milliseconds: (simPath.states.last.timeSeconds * 1000).toInt());
          _previewController.repeat();
        } else {
          double prevTime = _previewController.value *
              (_previewController.duration!.inMilliseconds / 1000.0);
          _previewController.duration = Duration(
              milliseconds: (simPath.states.last.timeSeconds * 1000).toInt());
          double newPos = prevTime / simPath.states.last.timeSeconds;
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

  void _showGenerationFailedError() {
    Log.warning('Failed to generate trajectory for auto: ${widget.auto.name}');

    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(
          'Failed to generate trajectory for ${widget.auto.name}. This is likely due to bad control point placement. Please adjust your control points to avoid kinks in the path.',
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
}
