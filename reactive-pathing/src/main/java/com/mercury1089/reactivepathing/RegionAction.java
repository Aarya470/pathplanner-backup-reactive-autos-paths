package com.mercury1089.reactivepathing;

/** What the robot does when a region reacts. */
public enum RegionAction {
  /** Keep going on the main path (after waiting, or after a pickup) */
  CONTINUE,

  /** Leave the main path and follow the region's divert path */
  DIVERT;

  /** Read the name the app saves: "divert", or anything else for CONTINUE. */
  public static RegionAction fromName(String name) {
    if ("divert".equalsIgnoreCase(name)) {
      return DIVERT;
    }
    return CONTINUE;
  }
}
