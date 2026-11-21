import 'package:file/memory.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:pathplanner/path/obstacle_region.dart';
import 'package:pathplanner/path/pathplanner_path.dart';
import 'package:pathplanner/widgets/editor/tree_widgets/obstacle_regions_tree.dart';
import 'package:undo/undo.dart';

void main() {
  late ChangeStack undoStack;
  late PathPlannerPath path;
  int? selectedRegion;

  setUp(() {
    undoStack = ChangeStack();
    path =
        PathPlannerPath.defaultPath(pathDir: '/paths', fs: MemoryFileSystem());
    path.obstacleRegions = [
      ObstacleRegion(
          startPos: 0.2, endPos: 0.8, trigger: RegionTrigger.gamePiece),
    ];
    selectedRegion = null;
  });

  Future<void> openRegion(WidgetTester tester) async {
    await tester.pumpWidget(MaterialApp(
      home: Scaffold(
        body: SingleChildScrollView(
          child: ObstacleRegionsTree(
            path: path,
            undoStack: undoStack,
            onRegionSelected: (value) => selectedRegion = value,
          ),
        ),
      ),
    ));
    await tester.tap(find.text('Path Regions'));
    await tester.pumpAndSettle();
    await tester.tap(find.textContaining('Region 1:'));
    await tester.pumpAndSettle();
  }

  testWidgets('choosing a zone makes a square zone on the region',
      (tester) async {
    await openRegion(tester);
    expect(find.text('Max distance to the piece (m)'), findsOneWidget);

    await tester.tap(find.text('Inside a zone'));
    await tester.pumpAndSettle();

    final zone = path.obstacleRegions[0].pickupZone;
    expect(zone, isNotNull);
    expect(zone!.length, 4);
    expect(selectedRegion, 0);
    expect(find.text('Max distance to the piece (m)'), findsNothing);
    expect(find.text('Re-center zone on this region'), findsOneWidget);
  });

  testWidgets(
      'going back to distance removes the zone, and undo brings it back',
      (tester) async {
    await openRegion(tester);
    await tester.tap(find.text('Inside a zone'));
    await tester.pumpAndSettle();

    await tester.tap(find.text('Within a distance'));
    await tester.pumpAndSettle();
    expect(path.obstacleRegions[0].pickupZone, isNull);

    undoStack.undo();
    await tester.pumpAndSettle();
    expect(path.obstacleRegions[0].pickupZone, isNotNull);
  });
}
