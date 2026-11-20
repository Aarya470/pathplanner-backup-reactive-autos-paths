package com.pathplanner.lib.obstacle;

/**
 * Decides what to do for one obstacle region, one loop at a time.
 *
 * <ul>
 *   <li>An obstacle seen inside the region arms it. The robot keeps driving through the region.
 *   <li>At the end of the region, with no timeout: divert (if the action is DIVERT), or keep going.
 *   <li>At the end of the region, with a timeout: stop for the timeout. Then, if the obstacle is
 *       still there and the action is DIVERT, divert. Otherwise resume the main path.
 * </ul>
 *
 * <p>Once a region has diverted or resumed, it is done for this run.
 */
public final class RegionMonitor {

  /** What the robot should do this loop */
  public enum Decision {
    /** Keep following the main path */
    NONE,
    /** Stop and wait for the timeout */
    STOP,
    /** Continue along the main path from where the robot is */
    RESUME,
    /** Leave the main path and follow the diverge path */
    DIVERT,
    /** Leave the main path to pick up a game piece (game piece regions only) */
    PICKUP
  }

  private final Double timeoutSec;
  private final RegionAction action;
  private boolean armed = false;
  private boolean waiting = false;
  private boolean done = false;
  private double waitStartSec = 0.0;

  /**
   * @param timeoutSec seconds to stop and wait at the end of the region, or null for no wait
   * @param action what to do if the obstacle is still there
   */
  public RegionMonitor(Double timeoutSec, RegionAction action) {
    this.timeoutSec = timeoutSec;
    this.action = action;
  }

  /**
   * Update the region with this loop's readings. The robot keeps driving through the region after
   * an obstacle appears, and decides at the end of the region.
   *
   * @param inRegion the robot is inside the region
   * @param pastEnd the robot has reached the end of the region
   * @param blocked an obstacle that counts for this region is near the region right now
   * @param nowSec the current time, seconds
   * @return what the robot should do
   */
  public Decision update(boolean inRegion, boolean pastEnd, boolean blocked, double nowSec) {
    if (done) return Decision.NONE;

    if (waiting) {
      if (nowSec - waitStartSec < timeoutSec) return Decision.STOP;

      waiting = false;
      done = true;
      if (blocked && action == RegionAction.DIVERT) return Decision.DIVERT;
      return Decision.RESUME;
    }

    if (!armed) {
      if (inRegion && blocked) {
        armed = true;
      } else {
        return Decision.NONE;
      }
    }

    if (!pastEnd) return Decision.NONE;

    // At the end of the region
    if (timeoutSec == null) {
      done = true;
      return action == RegionAction.DIVERT ? Decision.DIVERT : Decision.NONE;
    }

    waiting = true;
    waitStartSec = nowSec;
    return Decision.STOP;
  }

  /** @return true once this region has diverted or resumed */
  public boolean isDone() {
    return done;
  }

  /** Clear all state. Call at the start of each run. */
  public void reset() {
    armed = false;
    waiting = false;
    done = false;
    waitStartSec = 0.0;
  }
}
