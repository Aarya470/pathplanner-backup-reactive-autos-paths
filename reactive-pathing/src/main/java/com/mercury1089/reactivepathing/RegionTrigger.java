package com.mercury1089.reactivepathing;

/** What makes a region react. */
public enum RegionTrigger {
  /** Another robot in this part of the path */
  ROBOT,

  /** A game piece inside the region's pickup zone */
  GAME_PIECE;

  /** Read the name the app saves: "gamePiece", or anything else for ROBOT. */
  public static RegionTrigger fromName(String name) {
    if ("gamePiece".equals(name)) {
      return GAME_PIECE;
    }
    return ROBOT;
  }
}
