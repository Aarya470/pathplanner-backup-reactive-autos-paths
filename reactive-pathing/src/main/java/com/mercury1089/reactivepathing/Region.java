package com.mercury1089.reactivepathing;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

/**
 * One region drawn on a path in the app: a part of the path to watch, and what to do there.
 *
 * <p>Positions along the path are "waypoint relative": 0 is the first waypoint, 1 is the second,
 * 1.5 is halfway between the second and third, and so on. That is how PathPlanner measures them.
 */
public class Region {
  private final double startPos;
  private final double endPos;
  private final RegionTrigger trigger;
  private final RegionTeam team;
  private final double waitSeconds;
  private final RegionAction action;
  private final PathPlannerPath divertPath;
  private final List<Translation2d> pickupZone;
  private final double maxPieceDistance;

  /**
   * @param startPos where the region starts along the path
   * @param endPos where the region ends along the path
   * @param trigger ROBOT or GAME_PIECE
   * @param team which robots count (robot regions only)
   * @param waitSeconds how long to stop and wait for a robot to move, or 0 for no wait
   * @param action CONTINUE or DIVERT
   * @param divertPath the path to divert onto, or null
   * @param pickupZone the corners of the pickup zone (game piece regions), or an empty list
   * @param maxPieceDistance for old game piece regions with no zone: how close a piece must be
   */
  public Region(
      double startPos,
      double endPos,
      RegionTrigger trigger,
      RegionTeam team,
      double waitSeconds,
      RegionAction action,
      PathPlannerPath divertPath,
      List<Translation2d> pickupZone,
      double maxPieceDistance) {
    this.startPos = startPos;
    this.endPos = endPos;
    this.trigger = trigger;
    this.team = team;
    this.waitSeconds = waitSeconds;
    this.action = action;
    this.divertPath = divertPath;
    this.pickupZone = new ArrayList<>(pickupZone);
    this.maxPieceDistance = maxPieceDistance;
  }

  public double getStartPos() {
    return startPos;
  }

  public double getEndPos() {
    return endPos;
  }

  public RegionTrigger getTrigger() {
    return trigger;
  }

  public boolean isGamePieceRegion() {
    return trigger == RegionTrigger.GAME_PIECE;
  }

  public RegionTeam getTeam() {
    return team;
  }

  /** @return how long to wait for a robot to move, or 0 for no wait */
  public double getWaitSeconds() {
    return waitSeconds;
  }

  public RegionAction getAction() {
    return action;
  }

  /** @return the divert path, or null if there is none */
  public PathPlannerPath getDivertPath() {
    return divertPath;
  }

  /** @return the pickup zone's corners, or an empty list if there is none */
  public List<Translation2d> getPickupZone() {
    return pickupZone;
  }

  /** @return true if this region has a pickup zone (3 or more corners) */
  public boolean hasPickupZone() {
    return pickupZone.size() >= 3;
  }

  /**
   * @return for old game piece regions saved before zones existed: how close to the robot a piece
   *     must be. Opening and saving the path in the app gives it a zone.
   */
  public double getMaxPieceDistance() {
    return maxPieceDistance;
  }

  /** @return true if a path position is inside this region */
  public boolean contains(double pos) {
    return pos >= startPos && pos <= endPos;
  }
}
