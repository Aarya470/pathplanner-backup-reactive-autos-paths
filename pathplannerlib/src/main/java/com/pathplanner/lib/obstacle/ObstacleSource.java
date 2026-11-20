package com.pathplanner.lib.obstacle;

import java.util.List;

/**
 * Anything that can report robots on the field: a camera, a radio tracker, or a test fake. Teams
 * add their own by implementing this interface.
 */
public interface ObstacleSource {
  /**
   * Get the robots seen most recently. Called once per loop.
   *
   * @return the latest obstacles, possibly empty. Never null.
   */
  List<Obstacle> getObstacles();
}
