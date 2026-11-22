package com.mercury1089.reactivepathing;

import com.pathplanner.lib.path.PathPlannerPath;
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
 * Reads the regions the app saved in a path file. PathPlannerLib ignores this part of the file, so
 * it is read here.
 */
public class RegionFile {
  /**
   * How close a piece must be for old game piece regions with no zone, if the file doesn't say.
   * The same as the app's default.
   */
  static final double DEFAULT_MAX_PIECE_DISTANCE = 2.0;

  /** What was read from each file, so each file is only read once */
  private static final Map<String, PathFileData> cache = new HashMap<>();

  private RegionFile() {}

  /** The regions of a path and where its waypoints are, as saved in the file. */
  public static class PathFileData {
    private final List<Region> regions;
    private final List<Translation2d> waypoints;

    PathFileData(List<Region> regions, List<Translation2d> waypoints) {
      this.regions = regions;
      this.waypoints = waypoints;
    }

    public List<Region> getRegions() {
      return regions;
    }

    /**
     * @return where each waypoint is in the file (its anchor), used to tell if an auto mirrored
     *     the path
     */
    public List<Translation2d> getWaypoints() {
      return waypoints;
    }
  }

  /**
   * Read a path's regions from deploy/pathplanner/paths/[name].path.
   *
   * @return the data, or data with no regions if the file can't be read
   */
  public static PathFileData read(String pathName) {
    if (cache.containsKey(pathName)) {
      return cache.get(pathName);
    }
    PathFileData data = new PathFileData(new ArrayList<>(), new ArrayList<>());
    File file = new File(Filesystem.getDeployDirectory(), "pathplanner/paths/" + pathName + ".path");
    try {
      String text = Files.readString(file.toPath());
      JSONObject json = (JSONObject) new JSONParser().parse(text);
      data = fromJson(json);
    } catch (Exception e) {
      DriverStation.reportWarning(
          "Reactive Pathing: could not read regions from " + file + ": " + e.getMessage(), false);
    }
    cache.put(pathName, data);
    return data;
  }

  /** Read the regions from a path file's data. */
  public static PathFileData fromJson(JSONObject pathJson) {
    List<Region> regions = new ArrayList<>();
    for (Object item : PathFileReader.listOrEmpty(pathJson.get("obstacleRegions"))) {
      regions.add(regionFromJson((JSONObject) item));
    }

    List<Translation2d> waypoints = new ArrayList<>();
    JSONArray waypointList = PathFileReader.listOrEmpty(pathJson.get("waypoints"));
    for (Object item : waypointList) {
      JSONObject waypoint = (JSONObject) item;
      waypoints.add(PathFileReader.translation((JSONObject) waypoint.get("anchor")));
    }
    return new PathFileData(regions, waypoints);
  }

  static Region regionFromJson(JSONObject json) {
    double waitSeconds = 0.0;
    if (json.get("blockTimeoutSec") != null) {
      waitSeconds = PathFileReader.number(json.get("blockTimeoutSec"));
    }

    PathPlannerPath divertPath = null;
    if (json.get("divertPath") != null) {
      divertPath = PathFileReader.pathFromJson((JSONObject) json.get("divertPath"));
    }

    List<Translation2d> zone = new ArrayList<>();
    for (Object item : PathFileReader.listOrEmpty(json.get("pickupZone"))) {
      zone.add(PathFileReader.translation((JSONObject) item));
    }

    double maxPieceDistance = DEFAULT_MAX_PIECE_DISTANCE;
    if (json.get("pieceMaxDistance") != null) {
      maxPieceDistance = PathFileReader.number(json.get("pieceMaxDistance"));
    }

    return new Region(
        PathFileReader.number(json.get("startPos")),
        PathFileReader.number(json.get("endPos")),
        RegionTrigger.fromName((String) json.get("trigger")),
        RegionTeam.fromName((String) json.get("obstacleTeam")),
        waitSeconds,
        RegionAction.fromName((String) json.get("action")),
        divertPath,
        zone,
        maxPieceDistance);
  }
}
