package com.pathplanner.lib.obstacle;

/** Which robots on the field count as obstacles for a backup branch. */
public enum ObstacleTeam {
  OPPONENT,
  TEAMMATE,
  ANY;

  /**
   * Parse the team name from the path file. Case-insensitive. Anything unknown means ANY.
   *
   * @param name the name to parse
   * @return the matching team
   */
  public static ObstacleTeam fromString(String name) {
    if (name == null) return ANY;
    switch (name.toLowerCase()) {
      case "opponent":
        return OPPONENT;
      case "teammate":
        return TEAMMATE;
      default:
        return ANY;
    }
  }
}
