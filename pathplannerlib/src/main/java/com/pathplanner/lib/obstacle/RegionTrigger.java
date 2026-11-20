package com.pathplanner.lib.obstacle;

/** What a region watches for. */
public enum RegionTrigger {
  /** Another robot in the way of the path */
  ROBOT,
  /** A game piece close to the robot, to go pick up */
  GAME_PIECE;

  /**
   * Parse the name saved by the editor. Anything else is ROBOT, so older path files keep working.
   *
   * @param name "robot" or "gamePiece"
   * @return the matching trigger
   */
  public static RegionTrigger fromString(String name) {
    return "gamePiece".equals(name) ? GAME_PIECE : ROBOT;
  }
}
