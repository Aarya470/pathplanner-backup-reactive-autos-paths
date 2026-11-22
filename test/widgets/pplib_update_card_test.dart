import 'package:file/memory.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:mockito/annotations.dart';
import 'package:mockito/mockito.dart';
import 'package:pathplanner/services/update_checker.dart';
import 'package:pathplanner/widgets/pplib_update_card.dart';

import '../test_helpers.dart';
import 'pplib_update_card_test.mocks.dart';

@GenerateNiceMocks([MockSpec<UpdateChecker>()])
void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  Future<void> showCard(
      WidgetTester widgetTester, RobotLibraryUpdate update) async {
    FlutterError.onError = ignoreOverflowErrors;

    var updateChecker = MockUpdateChecker();
    when(updateChecker.checkRobotLibrary(
            projectDir: anyNamed('projectDir'), fs: anyNamed('fs')))
        .thenAnswer((realInvocation) => Future.value(update));

    var fs = MemoryFileSystem();
    await widgetTester.pumpWidget(MaterialApp(
      home: Scaffold(
        body: PPLibUpdateCard(
          projectDir: fs.directory('/project'),
          fs: fs,
          updateChecker: updateChecker,
        ),
      ),
    ));
  }

  testWidgets('pplib update card', (widgetTester) async {
    await showCard(widgetTester, RobotLibraryUpdate.newerVersion);

    // Card initially hidden
    expect(find.text('Robot library update available!'), findsNothing);

    await widgetTester.pumpAndSettle();

    expect(find.text('Robot library update available!'), findsOneWidget);
    expect(find.text('Dismiss'), findsOneWidget);

    await widgetTester.tap(find.text('Dismiss'));
    await widgetTester.pumpAndSettle();

    expect(find.text('Robot library update available!'), findsNothing);
  });

  testWidgets('pplib update card tells 1.x projects what to do',
      (widgetTester) async {
    await showCard(widgetTester, RobotLibraryUpdate.oldLibrary);
    await widgetTester.pumpAndSettle();

    expect(find.textContaining('uses the 1.x robot library'), findsOneWidget);
    expect(
        find.textContaining('PathplannerLib-BackupPaths.json'), findsOneWidget);
  });

  testWidgets('pplib update card stays hidden when up to date',
      (widgetTester) async {
    await showCard(widgetTester, RobotLibraryUpdate.none);
    await widgetTester.pumpAndSettle();

    expect(find.text('Dismiss'), findsNothing);
  });
}
