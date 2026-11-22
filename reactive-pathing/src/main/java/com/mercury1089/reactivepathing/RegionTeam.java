package com.mercury1089.reactivepathing;

/** Which robots count for a "robot in the way" region. */
public enum RegionTeam {
  /** Any robot */
  ANY,

  /** Only robots on the other alliance */
  OPPONENT,

  /** Only robots on our alliance */
  TEAMMATE;

  /** Read the name the app saves: "opponent", "teammate", or anything else for ANY. */
  public static RegionTeam fromName(String name) {
    if (name == null) {
      return ANY;
    }
    if (name.equalsIgnoreCase("opponent")) {
      return OPPONENT;
    }
    if (name.equalsIgnoreCase("teammate")) {
      return TEAMMATE;
    }
    return ANY;
  }
}
