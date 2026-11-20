package com.pathplanner.lib.obstacle;

/** Alliance color, from the FMS or from an obstacle source. */
public enum AllianceColor {
  RED,
  BLUE,
  UNKNOWN;

  /**
   * Parse a color name from the path file or config. Case-insensitive. Anything else is UNKNOWN.
   *
   * @param name the name to parse
   * @return the matching color
   */
  public static AllianceColor fromString(String name) {
    if (name == null) return UNKNOWN;
    switch (name.toLowerCase()) {
      case "red":
        return RED;
      case "blue":
        return BLUE;
      default:
        return UNKNOWN;
    }
  }
}
