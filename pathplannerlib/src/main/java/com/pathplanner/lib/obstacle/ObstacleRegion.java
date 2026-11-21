package com.pathplanner.lib.obstacle;

import com.pathplanner.lib.path.PathPlannerPath;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Function;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

/**
 * A part of a path watched for another robot or for a game piece, as saved by the PathPlanner
 * editor.
 *
 * <p>Start and end use the same scale as rotation targets: 0 is the first waypoint and the last
 * waypoint is the number of waypoints minus one.
 */
public class ObstacleRegion {
  private final double startPos;
  private final double endPos;
  private final ObstacleTeam team;
  private final Double timeoutSec;
  private final RegionAction action;
  private final PathPlannerPath divertPath;
  private final RegionTrigger trigger;
  private final double pieceMaxDistanceM;
  private final List<Point> pickupZone;

  /** Default for how close a game piece must be to the robot to go get it, meters */
  public static final double DEFAULT_PIECE_MAX_DISTANCE_M = 2.0;

  /**
   * @param startPos where the region starts on the path
   * @param endPos where the region ends on the path
   * @param team which robots count as obstacles
   * @param timeoutSec seconds to wait before deciding, or null to decide right away
   * @param action what to do if the obstacle is still there
   * @param divertPath the diverge path, or null
   */
  public ObstacleRegion(
      double startPos,
      double endPos,
      ObstacleTeam team,
      Double timeoutSec,
      RegionAction action,
      PathPlannerPath divertPath) {
    this(startPos, endPos, team, timeoutSec, action, divertPath, RegionTrigger.ROBOT,
        DEFAULT_PIECE_MAX_DISTANCE_M);
  }

  /**
   * @param startPos where the region starts on the path
   * @param endPos where the region ends on the path
   * @param team which robots count as obstacles (robot regions only)
   * @param timeoutSec seconds to wait before deciding, or null (robot regions only)
   * @param action robot regions: what to do if the robot is still there. Game piece regions: what
   *     to do after the pickup (CONTINUE rejoins the main path, DIVERT follows the diverge path).
   * @param divertPath the diverge path, or null
   * @param trigger what the region watches for
   * @param pieceMaxDistanceM game piece regions: how close a piece must be to the robot
   */
  public ObstacleRegion(
      double startPos,
      double endPos,
      ObstacleTeam team,
      Double timeoutSec,
      RegionAction action,
      PathPlannerPath divertPath,
      RegionTrigger trigger,
      double pieceMaxDistanceM) {
    this(startPos, endPos, team, timeoutSec, action, divertPath, trigger, pieceMaxDistanceM, null);
  }

  /**
   * Create a region with a pickup zone.
   *
   * @param startPos where the region starts on the path
   * @param endPos where the region ends on the path
   * @param team which robots count as obstacles
   * @param timeoutSec seconds to wait before deciding, or null to decide right away
   * @param action what to do when the region triggers
   * @param divertPath the diverge path, or null
   * @param trigger what the region watches for
   * @param pieceMaxDistanceM how close a game piece must be, when there is no pickup zone
   * @param pickupZone corners of the pickup zone in blue-side field meters, or null for none. A
   *     game piece is picked up only if it is inside this zone.
   */
  public ObstacleRegion(
      double startPos,
      double endPos,
      ObstacleTeam team,
      Double timeoutSec,
      RegionAction action,
      PathPlannerPath divertPath,
      RegionTrigger trigger,
      double pieceMaxDistanceM,
      List<Point> pickupZone) {
    this.pickupZone =
        pickupZone == null || pickupZone.size() < 3
            ? List.of()
            : Collections.unmodifiableList(new ArrayList<>(pickupZone));
    this.trigger = trigger;
    this.pieceMaxDistanceM = pieceMaxDistanceM;
    this.startPos = startPos;
    this.endPos = endPos;
    this.team = team;
    this.timeoutSec = timeoutSec;
    this.action = action;
    this.divertPath = divertPath;
  }

  /**
   * Read one region from a path file.
   *
   * @param json one entry of "obstacleRegions"
   * @param pathParser builds the diverge path from its JSON
   * @return the region
   */
  public static ObstacleRegion fromJson(
      JSONObject json, Function<JSONObject, PathPlannerPath> pathParser) {
    Object timeout = json.get("blockTimeoutSec");
    JSONObject divert = (JSONObject) json.get("divertPath");

    return new ObstacleRegion(
        number(json.get("startPos")),
        number(json.get("endPos")),
        ObstacleTeam.fromString((String) json.get("obstacleTeam")),
        timeout == null ? null : number(timeout),
        RegionAction.fromString((String) json.get("action")),
        divert == null ? null : pathParser.apply(divert),
        RegionTrigger.fromString((String) json.get("trigger")),
        json.get("pieceMaxDistance") == null
            ? DEFAULT_PIECE_MAX_DISTANCE_M
            : number(json.get("pieceMaxDistance")),
        zoneFromJson(json.get("pickupZone")));
  }

  private static List<Point> zoneFromJson(Object value) {
    if (!(value instanceof JSONArray)) return null;
    List<Point> corners = new ArrayList<>();
    for (Object corner : (JSONArray) value) {
      if (corner instanceof JSONObject) {
        JSONObject c = (JSONObject) corner;
        corners.add(new Point(number(c.get("x")), number(c.get("y"))));
      }
    }
    return corners;
  }

  private static double number(Object value) {
    return value == null ? 0.0 : ((Number) value).doubleValue();
  }

  /** @return where the region starts on the path */
  public double getStartPos() {
    return startPos;
  }

  /** @return where the region ends on the path */
  public double getEndPos() {
    return endPos;
  }

  /** @return which robots count as obstacles */
  public ObstacleTeam getTeam() {
    return team;
  }

  /** @return seconds to wait before deciding, or null to decide right away */
  public Double getTimeoutSec() {
    return timeoutSec;
  }

  /** @return what to do if the obstacle is still there */
  public RegionAction getAction() {
    return action;
  }

  /** @return the diverge path, or null */
  public PathPlannerPath getDivertPath() {
    return divertPath;
  }

  /** @return what the region watches for */
  public RegionTrigger getTrigger() {
    return trigger;
  }

  /** @return game piece regions: how close a piece must be to the robot, meters */
  public double getPieceMaxDistanceM() {
    return pieceMaxDistanceM;
  }

  /**
   * Corners of the pickup zone, in blue-side field meters.
   *
   * @return the corners, or an empty list if this region has no pickup zone
   */
  public List<Point> getPickupZone() {
    return pickupZone;
  }

  /**
   * Whether game pieces are chosen by the pickup zone instead of the max distance.
   *
   * @return true if this region has a pickup zone
   */
  public boolean hasPickupZone() {
    return !pickupZone.isEmpty();
  }
}
