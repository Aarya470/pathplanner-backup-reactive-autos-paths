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

  testWidgets('a game piece region always gets a zone, with no distance option',
      (tester) async {
    await openRegion(tester);
    final zone = path.obstacleRegions[0].pickupZone;
    expect(zone, isNotNull);
    expect(zone!.length, 4);
    expect(find.text('Within a distance'), findsNothing);
    expect(find.text('Max distance to the piece (m)'), findsNothing);
    expect(find.text('Reset zone around this region'), findsOneWidget);
  });

  testWidgets('switching a region to game piece makes a zone, undo removes it',
      (tester) async {
    path.obstacleRegions = [ObstacleRegion(startPos: 0.2, endPos: 0.8)];
    await openRegion(tester);
    expect(path.obstacleRegions[0].pickupZone, isNull);

    await tester.tap(find.text('Game piece'));
    await tester.pumpAndSettle();
    expect(path.obstacleRegions[0].pickupZone, isNotNull);
    expect(selectedRegion, 0);

    undoStack.undo();
    await tester.pumpAndSettle();
    expect(path.obstacleRegions[0].trigger, RegionTrigger.robot);
  });

  testWidgets('adding a region opens and selects it', (tester) async {
    path.obstacleRegions = [];
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
    await tester.tap(find.byTooltip('Add obstacle region'));
    await tester.pumpAndSettle();
    expect(path.obstacleRegions.length, 1);
    expect(selectedRegion, 0);
    expect(find.text('Part of the path to watch'), findsOneWidget);
  });
}
