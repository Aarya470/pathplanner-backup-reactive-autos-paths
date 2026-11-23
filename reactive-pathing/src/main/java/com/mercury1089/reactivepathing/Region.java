package com.mercury1089.reactivepathing;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

/**
 * One region drawn on a path in the app: a part of the path to watch, and what to do there.
 *
 * <p>Positions along a path are "waypoint relative": 0 is the first waypoint, 1 is the second, 1.5
 * is halfway between the second and third, and so on. That is how PathPlanner measures them.
 */
public class Region {
  /** What makes the region react */
  public enum Trigger {
    ROBOT,
    GAME_PIECE
  }

  /** Which robots count, for robot regions */
  public enum Team {
    ANY,
    OPPONENT,
    TEAMMATE
  }

  /** What to do when it reacts */
  public enum Action {
    CONTINUE, // keep going on the path (after waiting, or after a pickup)
    DIVERT // leave the path and follow the divert path
  }

  /** For game piece regions with no zone (saved by old versions of the app): how close a piece must be */
  static final double NO_ZONE_PIECE_DISTANCE = 2.0;

  final double startPos;
  final double endPos;
  final Trigger trigger;
  final Team team;
  final double waitSeconds; // how long to wait for a robot to move, or 0 to not wait
  final Action action;
  final PathPlannerPath divertPath; // null if there is none
  final List<Translation2d> pickupZone; // the zone's corners, drawn on the blue side

  Region(
      double startPos,
      double endPos,
      Trigger trigger,
      Team team,
      double waitSeconds,
      Action action,
      PathPlannerPath divertPath,
      List<Translation2d> pickupZone) {
    this.startPos = startPos;
    this.endPos = endPos;
    this.trigger = trigger;
    this.team = team;
    this.waitSeconds = waitSeconds;
    this.action = action;
    this.divertPath = divertPath;
    this.pickupZone = pickupZone;
  }

  /** @return true if a position along the path is inside this region */
  boolean contains(double pos) {
    return pos >= startPos && pos <= endPos;
  }

  // ---------------------------------------------------------------- reading regions from a path

  // Each path file is only read from storage once
  private static final Map<String, JSONObject> fileCache = new HashMap<>();

  /**
   * The regions of a path, read from its file in deploy/pathplanner/paths. If an auto mirrored the
   * path, the regions are mirrored too.
   *
   * @return the regions, or an empty list if there are none
   */
  static List<Region> forPath(PathPlannerPath path) {
    JSONObject json = readPathFile(path.name);
    if (json == null) {
      return new ArrayList<>();
    }
    try {
      List<Region> regions = fromJson(json);
      if (isMirrored(path, json)) {
        regions = mirror(regions);
      }
      return regions;
    } catch (Exception e) {
      // A broken region shouldn't crash the robot: run the path with no regions instead
      DriverStation.reportWarning(
          "Reactive Pathing: could not read the regions of " + path.name + ": " + e, false);
      return new ArrayList<>();
    }
  }

  private static JSONObject readPathFile(String pathName) {
    if (fileCache.containsKey(pathName)) {
      return fileCache.get(pathName);
    }
    JSONObject json = null;
    File file = new File(Filesystem.getDeployDirectory(), "pathplanner/paths/" + pathName + ".path");
    try {
      json = (JSONObject) new JSONParser().parse(Files.readString(file.toPath()));
    } catch (Exception e) {
      DriverStation.reportWarning("Reactive Pathing: could not read " + file + ": " + e, false);
    }
    fileCache.put(pathName, json);
    return json;
  }

  /** Read the regions the app saved in a path file. */
  static List<Region> fromJson(JSONObject pathJson) {
    List<Region> regions = new ArrayList<>();
    for (Object item : listOf(pathJson.get("obstacleRegions"))) {
      JSONObject json = (JSONObject) item;

      Trigger trigger = Trigger.ROBOT;
      if ("gamePiece".equals(json.get("trigger"))) {
        trigger = Trigger.GAME_PIECE;
      }

      Team team = Team.ANY;
      if ("opponent".equals(json.get("obstacleTeam"))) {
        team = Team.OPPONENT;
      } else if ("teammate".equals(json.get("obstacleTeam"))) {
        team = Team.TEAMMATE;
      }

      Action action = Action.CONTINUE;
      if ("divert".equals(json.get("action"))) {
        action = Action.DIVERT;
      }

      double waitSeconds = 0;
      if (json.get("blockTimeoutSec") != null) {
        waitSeconds = number(json.get("blockTimeoutSec"));
      }

      PathPlannerPath divertPath = null;
      if (json.get("divertPath") != null) {
        divertPath = pathFromJson((JSONObject) json.get("divertPath"));
      }

      List<Translation2d> zone = new ArrayList<>();
      for (Object corner : listOf(json.get("pickupZone"))) {
        zone.add(point((JSONObject) corner));
      }

      regions.add(
          new Region(
              number(json.get("startPos")),
              number(json.get("endPos")),
              trigger,
              team,
              waitSeconds,
              action,
              divertPath,
              zone));
    }
    return regions;
  }

  /**
   * Make a path from path data saved in the file (the divert paths are saved inside their region).
   * PathPlannerLib's hotReload reads path data exactly like a .path file, so we make a placeholder
   * path and load the data into it.
   */
  static PathPlannerPath pathFromJson(JSONObject json) {
    PathPlannerPath path =
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(new Pose2d(), new Pose2d(1, 0, Rotation2d.kZero)),
            new PathConstraints(1, 1, 1, 1),
            null,
            new GoalEndState(0, Rotation2d.kZero));
    path.hotReload(json);
    return path;
  }

  /**
   * An auto can mirror a path to the other side of the field. Then every waypoint is at the mirror
   * of where the file says it is.
   */
  static boolean isMirrored(PathPlannerPath path, JSONObject pathJson) {
    JSONArray waypointsInFile = listOf(pathJson.get("waypoints"));
    if (waypointsInFile.size() != path.getWaypoints().size()) {
      return false;
    }
    boolean allSame = true;
    boolean allMirrored = true;
    for (int i = 0; i < waypointsInFile.size(); i++) {
      JSONObject waypoint = (JSONObject) waypointsInFile.get(i);
      Translation2d inFile = point((JSONObject) waypoint.get("anchor"));
      Translation2d inPath = path.getWaypoints().get(i).anchor();
      if (inPath.getDistance(inFile) > 1e-6) {
        allSame = false;
      }
      if (inPath.getDistance(PathTools.mirror(inFile)) > 1e-6) {
        allMirrored = false;
      }
    }
    // A path drawn right on the middle line is the same either way: treat it as not mirrored
    return allMirrored && !allSame;
  }

  /** The same regions, with zones and divert paths mirrored to the other side of the field. */
  static List<Region> mirror(List<Region> regions) {
    List<Region> mirrored = new ArrayList<>();
    for (Region region : regions) {
      List<Translation2d> zone = new ArrayList<>();
      for (Translation2d corner : region.pickupZone) {
        zone.add(PathTools.mirror(corner));
      }
      PathPlannerPath divert = region.divertPath;
      if (divert != null) {
        divert = divert.mirrorPath();
      }
      mirrored.add(
          new Region(
              region.startPos,
              region.endPos,
              region.trigger,
              region.team,
              region.waitSeconds,
              region.action,
              divert,
              zone));
    }
    return mirrored;
  }

  private static Translation2d point(JSONObject json) {
    return new Translation2d(number(json.get("x")), number(json.get("y")));
  }

  private static double number(Object value) {
    return ((Number) value).doubleValue();
  }

  private static JSONArray listOf(Object value) {
    if (value instanceof JSONArray) {
      return (JSONArray) value;
    }
    return new JSONArray();
  }
}
