package com.pathplanner.lib.obstacle;

/** What an obstacle region does when an obstacle is still in it after the timeout */
public enum RegionAction {
  /** Keep following the main path */
  CONTINUE,
  /** Leave the main path and follow the diverge path */
  DIVERT;

  /**
   * Parse the action from the path file. Anything unknown means CONTINUE.
   *
   * @param name the name from the file
   * @return the matching action
   */
  public static RegionAction fromString(String name) {
    if ("divert".equalsIgnoreCase(name)) return DIVERT;
    return CONTINUE;
  }
}
