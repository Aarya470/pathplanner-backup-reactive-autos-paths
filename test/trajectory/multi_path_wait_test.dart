import 'package:file/memory.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:pathplanner/path/pathplanner_path.dart';
import 'package:pathplanner/path/waypoint.dart';
import 'package:pathplanner/trajectory/auto_simulator.dart';
import 'package:pathplanner/trajectory/config.dart';
import 'package:pathplanner/trajectory/trajectory.dart';
import 'package:pathplanner/util/wpimath/geometry.dart';
import 'package:shared_preferences/shared_preferences.dart';

PathPlannerPath _line(
    String name, Translation2d a, Translation2d b, MemoryFileSystem fs) {
  final p = PathPlannerPath.defaultPath(pathDir: '/paths', fs: fs, name: name);
  final d = (b - a) * (1 / 3);
  p.waypoints = [
    Waypoint(anchor: a, nextControl: a + d),
    Waypoint(anchor: b, prevControl: b - d),
  ];
  p.generatePathPoints();
  return p;
}

void main() {
  late RobotConfig config;

  setUp(() async {
    SharedPreferences.setMockInitialValues({});
    config = RobotConfig.fromPrefs(await SharedPreferences.getInstance());
  });

  test('simulateAuto reports where each path starts', () {
    final fs = MemoryFileSystem();
    final a =
        _line('A', const Translation2d(4, 2), const Translation2d(6, 2), fs);
    final b =
        _line('B', const Translation2d(6, 2), const Translation2d(2, 2), fs);
    final starts = <int>[];
    final traj =
        AutoSimulator.simulateAuto([a, b], config, pathStarts: starts)!;
    expect(starts.length, 2);
    expect(starts[0], 0);
    expect(starts[1], greaterThan(0));
    expect(starts[1], lessThan(traj.states.length));
  });

  test('a wait searched only inside its own path lands in that path', () {
    final fs = MemoryFileSystem();
    // A drives 4,2 -> 6,2. B drives back 6,2 -> 2,2 and waits at 4,2, which A also passes.
    final a =
        _line('A', const Translation2d(4, 2), const Translation2d(6, 2), fs);
    final b =
        _line('B', const Translation2d(6, 2), const Translation2d(2, 2), fs);
    final starts = <int>[];
    final traj =
        AutoSimulator.simulateAuto([a, b], config, pathStarts: starts)!;
    final total = traj.getTotalTimeSeconds();

    final after = insertWaitAt(traj.states, b.samplePath(0.5), 2.0,
        startIndex: starts[1], endIndex: traj.states.length);
    expect(after - 3, greaterThanOrEqualTo(starts[1]));
    expect(traj.getTotalTimeSeconds(), closeTo(total + 2.0, 1e-6));
  });

  test('endIndex limits the search', () {
    final fs = MemoryFileSystem();
    final a =
        _line('A', const Translation2d(2, 2), const Translation2d(6, 2), fs);
    final traj = AutoSimulator.simulateAuto([a], config)!;
    final after =
        insertWaitAt(traj.states, const Translation2d(6, 2), 1.0, endIndex: 5);
    expect(after - 3, lessThan(5));
  });
}
