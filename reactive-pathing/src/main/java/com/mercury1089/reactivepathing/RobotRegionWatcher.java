package com.mercury1089.reactivepathing;

/**
 * The rules for one "robot in the way" region, checked every loop.
 *
 * <ul>
 *   <li>A robot seen in the region "arms" it. The robot keeps driving through the region.
 *   <li>At the end of the region, with no wait time: divert (if the action is DIVERT), or keep
 *       going.
 *   <li>At the end of the region, with a wait time: stop and wait. Then, if the robot is still
 *       there and the action is DIVERT, divert. Otherwise go on along the path.
 * </ul>
 *
 * <p>Once a region has diverted or gone on, it is done for this auto.
 */
public class RobotRegionWatcher {
  /** What to do this loop */
  public enum Decision {
    /** Keep following the path */
    KEEP_GOING,
    /** Stop and wait */
    STOP,
    /** Done waiting: go on along the path from where the robot is */
    RESUME,
    /** Leave the path and follow the divert path */
    DIVERT
  }

  private final double waitSeconds;
  private final RegionAction action;

  private boolean armed = false;
  private boolean waiting = false;
  private boolean done = false;
  private double waitStartTime = 0.0;

  /**
   * @param waitSeconds how long to wait at the end of the region, or 0 to not wait
   * @param action what to do if the robot is still there
   */
  public RobotRegionWatcher(double waitSeconds, RegionAction action) {
    this.waitSeconds = waitSeconds;
    this.action = action;
  }

  /**
   * @param inRegion our robot is inside the region
   * @param pastEnd our robot has reached the end of the region
   * @param blocked a robot that counts for this region is in the way right now
   * @param now the time, in seconds
   * @return what to do
   */
  public Decision update(boolean inRegion, boolean pastEnd, boolean blocked, double now) {
    if (done) {
      return Decision.KEEP_GOING;
    }

    if (waiting) {
      if (now - waitStartTime < waitSeconds) {
        return Decision.STOP;
      }
      waiting = false;
      done = true;
      if (blocked && action == RegionAction.DIVERT) {
        return Decision.DIVERT;
      }
      return Decision.RESUME;
    }

    if (!armed) {
      if (inRegion && blocked) {
        armed = true;
      } else {
        return Decision.KEEP_GOING;
      }
    }

    if (!pastEnd) {
      return Decision.KEEP_GOING;
    }

    // At the end of the region
    if (waitSeconds <= 0) {
      done = true;
      if (action == RegionAction.DIVERT) {
        return Decision.DIVERT;
      }
      return Decision.KEEP_GOING;
    }
    waiting = true;
    waitStartTime = now;
    return Decision.STOP;
  }

  /** @return true once this region has diverted or gone on */
  public boolean isDone() {
    return done;
  }
}
