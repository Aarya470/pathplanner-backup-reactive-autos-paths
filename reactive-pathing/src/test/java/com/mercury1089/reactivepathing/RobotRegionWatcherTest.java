package com.mercury1089.reactivepathing;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.mercury1089.reactivepathing.RobotRegionWatcher.Decision;
import org.junit.jupiter.api.Test;

public class RobotRegionWatcherTest {
  @Test
  public void nothingHappensWithoutARobot() {
    RobotRegionWatcher w = new RobotRegionWatcher(2.0, RegionAction.DIVERT);
    assertEquals(Decision.KEEP_GOING, w.update(true, false, false, 0));
    assertEquals(Decision.KEEP_GOING, w.update(false, true, false, 1));
  }

  @Test
  public void aRobotSeenOutsideTheRegionDoesNotCount() {
    RobotRegionWatcher w = new RobotRegionWatcher(2.0, RegionAction.CONTINUE);
    assertEquals(Decision.KEEP_GOING, w.update(false, false, true, 0));
    assertEquals(Decision.KEEP_GOING, w.update(false, true, false, 1));
  }

  @Test
  public void waitsAtTheEndThenResumesWhenTheRobotLeaves() {
    RobotRegionWatcher w = new RobotRegionWatcher(2.0, RegionAction.DIVERT);
    assertEquals(Decision.KEEP_GOING, w.update(true, false, true, 0)); // armed, keeps driving
    assertEquals(Decision.STOP, w.update(false, true, true, 1)); // end of region: stop
    assertEquals(Decision.STOP, w.update(false, true, false, 2)); // still waiting
    assertEquals(Decision.RESUME, w.update(false, true, false, 3.1)); // gone: go on
    assertTrue(w.isDone());
    assertEquals(Decision.KEEP_GOING, w.update(true, false, true, 4)); // done for this auto
  }

  @Test
  public void divertsIfTheRobotIsStillThereAfterWaiting() {
    RobotRegionWatcher w = new RobotRegionWatcher(1.0, RegionAction.DIVERT);
    w.update(true, false, true, 0);
    assertEquals(Decision.STOP, w.update(false, true, true, 0.5));
    assertEquals(Decision.DIVERT, w.update(false, true, true, 1.6));
  }

  @Test
  public void continueActionResumesEvenIfTheRobotIsStillThere() {
    RobotRegionWatcher w = new RobotRegionWatcher(1.0, RegionAction.CONTINUE);
    w.update(true, false, true, 0);
    w.update(false, true, true, 0.5);
    assertEquals(Decision.RESUME, w.update(false, true, true, 1.6));
  }

  @Test
  public void noWaitDivertsRightAtTheEnd() {
    RobotRegionWatcher w = new RobotRegionWatcher(0, RegionAction.DIVERT);
    w.update(true, false, true, 0);
    assertEquals(Decision.DIVERT, w.update(false, true, false, 1));
  }

  @Test
  public void noWaitAndContinueJustKeepsGoing() {
    RobotRegionWatcher w = new RobotRegionWatcher(0, RegionAction.CONTINUE);
    w.update(true, false, true, 0);
    assertEquals(Decision.KEEP_GOING, w.update(false, true, true, 1));
    assertTrue(w.isDone());
  }
}
