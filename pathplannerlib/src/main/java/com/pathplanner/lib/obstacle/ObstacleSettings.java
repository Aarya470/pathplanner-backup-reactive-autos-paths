package com.pathplanner.lib.obstacle;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Global setup for obstacle regions. Call {@link #configure} once at robot startup. Any path
 * with obstacle regions in its file uses them, as long as a source is set.
 *
 * <p>Optional runtime controls (see {@link #setBackupOptions}) let the driver turn diverting and
 * waiting on or off, and pick the diverge path, from the dashboard. They are read each time a
 * region is run, so a change made before the path starts takes effect.
 */
public final class ObstacleSettings {
  private static ObstacleSource source = null;
  private static Supplier<Pose2d> poseSupplier = null;
  private static Runnable stopDrive = () -> {};
  private static double staleAfterSec = 0.25;

  private static BooleanSupplier divertEnabled = () -> true;
  private static BooleanSupplier waitEnabled = () -> true;
  private static Supplier<PathPlannerPath> divertOverride = () -> null;

  private ObstacleSettings() {}

  /**
   * @param obstacleSource where robots on the field come from
   * @param pose the robot pose from odometry
   * @param stop stops the drivetrain. Called while the robot waits during a timeout.
   * @param staleAfterSec obstacles older than this are ignored
   */
  public static void configure(
      ObstacleSource obstacleSource, Supplier<Pose2d> pose, Runnable stop, double staleAfterSec) {
    source = obstacleSource;
    poseSupplier = pose;
    stopDrive = stop;
    ObstacleSettings.staleAfterSec = staleAfterSec;
  }

  /**
   * Runtime controls for obstacle regions. Without this call, diverting and waiting are always on
   * and every region uses its own diverge path from the file.
   *
   * @param divert true to allow diverting. When false, every region acts as CONTINUE.
   * @param wait true to honor the timeout at the end of a region. When false, regions decide
   *     right at their end.
   * @param divertPath returns the diverge path to use for every region, or null to use each
   *     region's own diverge path from the file. Read only when a region diverts.
   */
  public static void setBackupOptions(
      BooleanSupplier divert, BooleanSupplier wait, Supplier<PathPlannerPath> divertPath) {
    divertEnabled = divert == null ? () -> true : divert;
    waitEnabled = wait == null ? () -> true : wait;
    divertOverride = divertPath == null ? () -> null : divertPath;
  }

  /**
   * @param action the action from the path file
   * @return the action to use now: CONTINUE if diverting is turned off, otherwise the file's
   */
  static RegionAction effectiveAction(RegionAction action) {
    return divertEnabled.getAsBoolean() ? action : RegionAction.CONTINUE;
  }

  /**
   * @param timeoutSec the timeout from the path file, or null
   * @return the timeout to use now: null (decide at the region end) if waiting is turned off
   */
  static Double effectiveTimeout(Double timeoutSec) {
    return waitEnabled.getAsBoolean() ? timeoutSec : null;
  }

  /** @return the diverge path picked on the dashboard, or null to use the region's own */
  static PathPlannerPath getDivertOverride() {
    return divertOverride.get();
  }

  /** @return the obstacle source, or null if not configured */
  public static ObstacleSource getSource() {
    return source;
  }

  /** @return the robot pose supplier, or null if not configured */
  public static Supplier<Pose2d> getPoseSupplier() {
    return poseSupplier;
  }

  /** @return the action that stops the drivetrain */
  public static Runnable getStopDrive() {
    return stopDrive;
  }

  /** @return how old an obstacle can be before it is ignored, seconds */
  public static double getStaleAfterSec() {
    return staleAfterSec;
  }
}
