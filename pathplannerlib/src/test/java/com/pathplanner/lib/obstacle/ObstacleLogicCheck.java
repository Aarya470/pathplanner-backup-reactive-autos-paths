package com.pathplanner.lib.obstacle;

import com.pathplanner.lib.obstacle.RegionMonitor.Decision;
import java.util.List;

/**
 * Plain checks for the region decision logic, geometry and dashboard options. No test framework.
 *
 * <p>Run from pathplannerlib/: compile all of src/main/java with WPILib on the classpath (the
 * options use Pose2d and PathPlannerPath), then compile this file against it and run java -ea on
 * com.pathplanner.lib.obstacle.ObstacleLogicCheck.
 */
public final class ObstacleLogicCheck {

  private static void check(boolean condition, String what) {
    if (!condition) throw new AssertionError(what);
  }

  static void testNoObstacleMeansNormalPath() {
    RegionMonitor m = new RegionMonitor(2.0, RegionAction.DIVERT);
    check(m.update(true, false, false, 0.0) == Decision.NONE, "no obstacle in region: nothing");
    check(m.update(false, true, false, 1.0) == Decision.NONE, "no obstacle at the end: nothing");
    check(!m.isDone(), "never armed, so not done");
  }

  static void testNoTimeoutDivertsAtRegionEnd() {
    RegionMonitor m = new RegionMonitor(null, RegionAction.DIVERT);
    check(m.update(true, false, true, 0.0) == Decision.NONE, "keep driving through the region");
    check(m.update(true, false, true, 0.5) == Decision.NONE, "still inside the region");
    check(m.update(false, true, true, 1.0) == Decision.DIVERT, "divert at the end of the region");
    check(m.isDone(), "done after diverting");
    check(m.update(false, true, true, 1.1) == Decision.NONE, "done regions do nothing");
  }

  static void testNoTimeoutContinueKeepsGoing() {
    RegionMonitor m = new RegionMonitor(null, RegionAction.CONTINUE);
    check(m.update(true, false, true, 0.0) == Decision.NONE, "keep driving");
    check(m.update(false, true, true, 1.0) == Decision.NONE, "continue at the end");
    check(m.isDone(), "done after the end");
  }

  static void testWaitsAtEndThenDivertsIfStillThere() {
    RegionMonitor m = new RegionMonitor(2.0, RegionAction.DIVERT);
    check(m.update(true, false, true, 0.0) == Decision.NONE, "armed, keep driving");
    check(m.update(false, true, true, 1.0) == Decision.STOP, "stop at the end of the region");
    check(m.update(false, true, true, 2.5) == Decision.STOP, "still waiting before 2 s");
    check(m.update(false, true, true, 3.0) == Decision.DIVERT, "still blocked after 2 s: divert");
    check(m.isDone(), "done after diverting");
  }

  static void testWaitsAtEndThenResumesIfGone() {
    RegionMonitor m = new RegionMonitor(2.0, RegionAction.DIVERT);
    m.update(true, false, true, 0.0);
    check(m.update(false, true, true, 1.0) == Decision.STOP, "stop at the end");
    check(m.update(false, true, false, 3.0) == Decision.RESUME, "gone after the wait: resume");
  }

  static void testObstacleGoneBeforeEndStillWaitsThenResumes() {
    RegionMonitor m = new RegionMonitor(2.0, RegionAction.DIVERT);
    m.update(true, false, true, 0.0);
    check(m.update(true, false, false, 0.5) == Decision.NONE, "gone inside the region: keep going");
    check(m.update(false, true, false, 1.0) == Decision.STOP, "still waits at the end");
    check(m.update(false, true, false, 3.0) == Decision.RESUME, "gone after the wait: resume");
  }

  static void testContinueWithTimeoutResumesEvenIfBlocked() {
    RegionMonitor m = new RegionMonitor(1.0, RegionAction.CONTINUE);
    m.update(true, false, true, 0.0);
    check(m.update(false, true, true, 1.0) == Decision.STOP, "stop at the end");
    check(m.update(false, true, true, 2.0) == Decision.RESUME, "continue action resumes");
  }

  static void testGeometryNearestIndex() {
    double[] xs = {0.0, 1.0, 2.0, 3.0};
    double[] ys = {0.0, 0.0, 0.0, 0.0};
    check(RegionGeometry.nearestIndex(xs, ys, 1.9, 0.5) == 2, "closest point should be index 2");
    check(RegionGeometry.nearestIndex(new double[0], new double[0], 0, 0) == -1, "no points gives -1");
  }

  static void testGeometryBlockedByOpponentOnly() {
    List<Point> regionPoints = List.of(new Point(2.0, 0.0), new Point(2.5, 0.0));
    List<Obstacle> blueBot = List.of(new Obstacle(new Point(2.2, 0.3), AllianceColor.BLUE, 10.0));
    check(
        RegionGeometry.blocked(blueBot, regionPoints, ObstacleTeam.OPPONENT, AllianceColor.RED,
            10.0, 0.25, 0.6),
        "blue robot 0.3 m from path should block an opponent region when we are red");
    check(
        !RegionGeometry.blocked(blueBot, regionPoints, ObstacleTeam.TEAMMATE, AllianceColor.RED,
            10.0, 0.25, 0.6),
        "blue robot should not block a teammate-only region when we are red");
    check(
        !RegionGeometry.blocked(blueBot, regionPoints, ObstacleTeam.OPPONENT, AllianceColor.UNKNOWN,
            10.0, 0.25, 0.6),
        "unknown alliance should not block an opponent-only region");
  }

  static void testGeometryIgnoresStaleAndFarObstacles() {
    List<Point> regionPoints = List.of(new Point(2.0, 0.0));
    List<Obstacle> old = List.of(new Obstacle(new Point(2.0, 0.1), AllianceColor.BLUE, 0.0));
    check(
        !RegionGeometry.blocked(old, regionPoints, ObstacleTeam.ANY, AllianceColor.RED, 5.0, 0.25, 0.6),
        "stale obstacle should be ignored");
    List<Obstacle> far = List.of(new Obstacle(new Point(2.0, 2.0), AllianceColor.BLUE, 5.0));
    check(
        !RegionGeometry.blocked(far, regionPoints, ObstacleTeam.ANY, AllianceColor.RED, 5.0, 0.25, 0.6),
        "far obstacle should be ignored");
  }

  public static void main(String[] args) {
    testNoObstacleMeansNormalPath();
    testNoTimeoutDivertsAtRegionEnd();
    testNoTimeoutContinueKeepsGoing();
    testWaitsAtEndThenDivertsIfStillThere();
    testWaitsAtEndThenResumesIfGone();
    testObstacleGoneBeforeEndStillWaitsThenResumes();
    testContinueWithTimeoutResumesEvenIfBlocked();
    testGeometryNearestIndex();
    testGeometryBlockedByOpponentOnly();
    testGeometryIgnoresStaleAndFarObstacles();
    testDefaultOptionsKeepFileBehavior();
    testDivertOffMakesEveryRegionContinue();
    testWaitOffDropsTimeout();
    System.out.println("obstacle region checks passed");
  }

  static void testDefaultOptionsKeepFileBehavior() {
    ObstacleSettings.setBackupOptions(null, null, null);
    check(ObstacleSettings.effectiveAction(RegionAction.DIVERT) == RegionAction.DIVERT,
        "default: file action DIVERT is kept");
    check(java.util.Objects.equals(ObstacleSettings.effectiveTimeout(2.0), 2.0),
        "default: file timeout is kept");
    check(ObstacleSettings.getDivertOverride() == null, "default: no divert override");
  }

  static void testDivertOffMakesEveryRegionContinue() {
    ObstacleSettings.setBackupOptions(() -> false, () -> true, () -> null);
    check(ObstacleSettings.effectiveAction(RegionAction.DIVERT) == RegionAction.CONTINUE,
        "divert off: DIVERT region becomes CONTINUE");
    check(ObstacleSettings.effectiveAction(RegionAction.CONTINUE) == RegionAction.CONTINUE,
        "divert off: CONTINUE stays CONTINUE");
    ObstacleSettings.setBackupOptions(() -> true, () -> true, () -> null);
    check(ObstacleSettings.effectiveAction(RegionAction.DIVERT) == RegionAction.DIVERT,
        "divert on: DIVERT region keeps DIVERT");
    ObstacleSettings.setBackupOptions(null, null, null);
  }

  static void testWaitOffDropsTimeout() {
    ObstacleSettings.setBackupOptions(() -> true, () -> false, () -> null);
    check(ObstacleSettings.effectiveTimeout(2.0) == null, "wait off: timeout is dropped");
    check(ObstacleSettings.effectiveTimeout(null) == null, "wait off: no timeout stays none");
    ObstacleSettings.setBackupOptions(() -> true, () -> true, () -> null);
    check(java.util.Objects.equals(ObstacleSettings.effectiveTimeout(2.0), 2.0),
        "wait on: timeout is kept");
    ObstacleSettings.setBackupOptions(null, null, null);
  }
}
