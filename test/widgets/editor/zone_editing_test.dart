import 'package:file/memory.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:pathplanner/commands/command_groups.dart';
import 'package:pathplanner/pages/project/project_page.dart';
import 'package:pathplanner/path/constraints_zone.dart';
import 'package:pathplanner/path/event_marker.dart';
import 'package:pathplanner/path/path_constraints.dart';
import 'package:pathplanner/path/pathplanner_path.dart';
import 'package:pathplanner/path/obstacle_region.dart';
import 'package:pathplanner/path/pickup_zone.dart';
import 'package:pathplanner/path/point_towards_zone.dart';
import 'package:pathplanner/path/rotation_target.dart';
import 'package:pathplanner/util/path_painter_util.dart';
import 'package:pathplanner/util/prefs.dart';
import 'package:pathplanner/util/wpimath/geometry.dart';
import 'package:pathplanner/widgets/editor/path_painter.dart';
import 'package:pathplanner/widgets/editor/split_path_editor.dart';
import 'package:pathplanner/widgets/field_image.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:undo/undo.dart';

void main() {
  late PathPlannerPath path;
  late SharedPreferences prefs;
  late ChangeStack undoStack;

  setUp(() async {
    var fs = MemoryFileSystem();
    await fs.directory('/paths').create(recursive: true);
    path = PathPlannerPath.defaultPath(
      name: 'path',
      pathDir: '/paths',
      fs: fs,
    );
    path.goalEndState.rotation = const Rotation2d();
    path.rotationTargets = [
      RotationTarget(0.5, const Rotation2d()),
    ];
    path.eventMarkers = [
      EventMarker(
        waypointRelativePos: 0.5,
        endWaypointRelativePos: 0.8,
        command: SequentialCommandGroup(commands: []),
        name: 'm',
      ),
    ];
    path.constraintZones = [
      ConstraintsZone(
        constraints: PathConstraints(),
        minWaypointRelativePos: 0.2,
        maxWaypointRelativePos: 0.8,
        name: 'z',
      ),
    ];
    path.pointTowardsZones = [
      PointTowardsZone(
        name: 'pz',
      ),
    ];
    undoStack = ChangeStack();
    SharedPreferences.setMockInitialValues({
      PrefsKeys.holonomicMode: true,
      PrefsKeys.treeOnRight: true,
      PrefsKeys.robotWidth: 1.0,
      PrefsKeys.robotLength: 1.0,
      PrefsKeys.showRobotDetails: true,
      PrefsKeys.showGrid: true,
    });
    prefs = await SharedPreferences.getInstance();
    ProjectPage.events.add('m');
  });

  Future<void> pumpEditor(WidgetTester t, FieldImage fieldImage) async {
    await t.pumpWidget(MaterialApp(
        home: Scaffold(
            body: SplitPathEditor(
                prefs: prefs,
                path: path,
                fieldImage: fieldImage,
                undoStack: undoStack))));
    await t.pumpAndSettle();
  }

  Offset cornerOnScreen(Translation2d c, FieldImage fieldImage) =>
      PathPainterUtil.pointToPixelOffset(c, PathPainter.scale, fieldImage) +
      const Offset(48, 48) +
      const Offset(0.0, 23.0);

  testWidgets(
      'dragging a zone corner is clamped, undoable, and leaves waypoints alone',
      (t) async {
    await t.binding.setSurfaceSize(const Size(1280, 720));
    final fieldImage = FieldImage.official(OfficialField.chargedUp);
    final region = ObstacleRegion(
        startPos: 0.3, endPos: 0.6, trigger: RegionTrigger.gamePiece);
    region.pickupZone = PickupZone.defaultZone(path, region);
    path.obstacleRegions = [region];
    final before = List.of(region.pickupZone!);
    final wpBefore = path.waypoints.map((w) => w.anchor).toList();
    await pumpEditor(t, fieldImage);

    final c0 = path.obstacleRegions[0].pickupZone![0];
    final g = await t.startGesture(cornerOnScreen(c0, fieldImage),
        kind: PointerDeviceKind.mouse);
    await t.pump();
    for (int i = 0; i < 400; i++) {
      await g.moveBy(const Offset(-1, 1));
      await t.pump();
    }
    await g.up();
    await t.pumpAndSettle();

    final z = path.obstacleRegions[0].pickupZone!;
    expect(z[0], isNot(c0));
    for (int i = 0; i < wpBefore.length; i++) {
      expect(path.waypoints[i].anchor, wpBefore[i]);
    }
    expect(
        PickupZone.clampCorner(path, path.obstacleRegions[0], z[0])
            .getDistance(z[0]),
        lessThan(1e-6));
    expect(PickupZone.isSimple(z), isTrue);

    undoStack.undo();
    await t.pumpAndSettle();
    expect(path.obstacleRegions[0].pickupZone, before);
    undoStack.redo();
    await t.pumpAndSettle();
    expect(path.obstacleRegions[0].pickupZone, z);
  });

  testWidgets('a corner cannot be dragged across the zone', (t) async {
    await t.binding.setSurfaceSize(const Size(1280, 720));
    final fieldImage = FieldImage.official(OfficialField.chargedUp);
    final region = ObstacleRegion(
        startPos: 0.3, endPos: 0.6, trigger: RegionTrigger.gamePiece);
    region.pickupZone = PickupZone.defaultZone(path, region);
    path.obstacleRegions = [region];
    await pumpEditor(t, fieldImage);

    final zone = path.obstacleRegions[0].pickupZone!;
    final from = cornerOnScreen(zone[0], fieldImage);
    final to = cornerOnScreen(zone[2], fieldImage);
    final g = await t.startGesture(from, kind: PointerDeviceKind.mouse);
    await t.pump();
    const steps = 60;
    for (int i = 0; i < steps; i++) {
      await g.moveBy((to - from) * (1.2 / steps));
      await t.pump();
    }
    await g.up();
    await t.pumpAndSettle();
    expect(PickupZone.isSimple(path.obstacleRegions[0].pickupZone!), isTrue);
  });

  testWidgets(
      'the view goes back to normal when its region changes or is deleted',
      (t) async {
    await t.binding.setSurfaceSize(const Size(1280, 720));
    final fieldImage = FieldImage.official(OfficialField.chargedUp);
    final pivot =
        PathPlannerPath.defaultPath(pathDir: '/paths', fs: path.fs, name: 'pv');
    pivot.isNested = true;
    path.obstacleRegions = [
      ObstacleRegion(
          startPos: 0.2,
          endPos: 0.5,
          action: RegionAction.divert,
          divertPath: pivot),
    ];
    await pumpEditor(t, fieldImage);
    await t.tap(find.text('Pivot'));
    await t.pumpAndSettle();

    PathPainter painter() => t
        .widget<CustomPaint>(find.byWidgetPredicate(
            (w) => w is CustomPaint && w.painter is PathPainter))
        .painter as PathPainter;
    expect(painter().wholePivot, isNotNull);
    expect(painter().showZoneHandles, isFalse);

    path.obstacleRegions[0].action = RegionAction.continuePath;
    await pumpEditor(t, fieldImage);
    expect(find.text('Pivot'), findsNothing);
    expect(painter().wholePivot, isNull);
    expect(painter().showZoneHandles, isTrue);

    // Back to a pivot, into Pivot view, then delete the region: no crash
    path.obstacleRegions[0].action = RegionAction.divert;
    await pumpEditor(t, fieldImage);
    await t.tap(find.text('Pivot'));
    await t.pumpAndSettle();
    path.obstacleRegions = [];
    await pumpEditor(t, fieldImage);
    expect(t.takeException(), isNull);
    expect(painter().wholePivot, isNull);
  });
}
