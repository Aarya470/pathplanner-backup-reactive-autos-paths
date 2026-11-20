import 'package:flutter_test/flutter_test.dart';
import 'package:pathplanner/trajectory/trajectory.dart';
import 'package:pathplanner/util/wpimath/geometry.dart';
import 'package:pathplanner/util/wpimath/kinematics.dart';

TrajectoryState at(num t, num x) {
  return TrajectoryState.pregen(
      t, const ChassisSpeeds(vx: 1), Pose2d(Translation2d(x, 0), const Rotation2d()));
}

void main() {
  test('Robot holds still for the timeout, then keeps moving', () {
    // Drives forward at 1 m/s from x=0, reaching x=1 at t=1 and x=2 at t=2
    final states = [at(0, 0), at(1, 1), at(2, 2)];
    insertWaitAt(states, const Translation2d(1, 0), 2);
    final traj = PathPlannerTrajectory.fromStates(states);

    // Still at x=1 during the whole 2 second wait
    final double a = traj.sample(1.2).pose.translation.x.toDouble();
    final double b = traj.sample(2.8).pose.translation.x.toDouble();
    expect(a, closeTo(1.0, 0.01));
    expect(b, closeTo(1.0, 0.01));
    // Moves again after the wait
    expect(traj.sample(3.5).pose.translation.x, greaterThan(1.05));
  });
}
