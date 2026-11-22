import 'package:file/memory.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:pathplanner/services/update_checker.dart';

void main() {
  test('a project still on the 1.x robot library needs an update', () async {
    final fs = MemoryFileSystem();
    final project = fs.directory('/robot')..createSync();
    fs.directory('/robot/vendordeps').createSync();
    fs
        .file('/robot/vendordeps/PathplannerLib-BackupPaths.json')
        .writeAsStringSync('{"version": "1.4.0"}');

    expect(await UpdateChecker().checkRobotLibrary(projectDir: project, fs: fs),
        RobotLibraryUpdate.oldLibrary);
  });

  test('a project without the robot library has nothing to update', () async {
    final fs = MemoryFileSystem();
    final project = fs.directory('/robot')..createSync();
    expect(await UpdateChecker().checkRobotLibrary(projectDir: project, fs: fs),
        RobotLibraryUpdate.none);
  });
}
