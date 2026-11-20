package com.pathplanner.lib.obstacle;

import java.util.List;

/** Anything that can report game pieces on the field: a camera or a test fake. */
public interface GamePieceSource {
  /**
   * Get the game pieces seen most recently. Can be called more than once per loop.
   *
   * @return the latest game pieces, possibly empty. Never null.
   */
  List<GamePiece> getGamePieces();
}
