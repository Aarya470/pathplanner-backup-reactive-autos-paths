package com.mercury1089.reactivepathing;

import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

/**
 * Builds a PathPlanner path from path file data. Used for the divert paths, which the app saves
 * inside a region. It reads the file the same way PathPlannerLib does.
 */
public class PathFileReader {
  private PathFileReader() {}

  /** Build a path from the data of a .path file. */
  public static PathPlannerPath pathFromJson(JSONObject json) {
    List<Waypoint> waypoints = new ArrayList<>();
    for (Object item : (JSONArray) json.get("waypoints")) {
      waypoints.add(waypointFromJson((JSONObject) item));
    }

    List<RotationTarget> rotationTargets = new ArrayList<>();
    for (Object item : listOrEmpty(json.get("rotationTargets"))) {
      JSONObject target = (JSONObject) item;
      double pos = number(target.get("waypointRelativePos"));
      double degrees = number(target.get("rotationDegrees"));
      rotationTargets.add(new RotationTarget(pos, Rotation2d.fromDegrees(degrees)));
    }

    List<PointTowardsZone> pointTowardsZones = new ArrayList<>();
    for (Object item : listOrEmpty(json.get("pointTowardsZones"))) {
      JSONObject zone = (JSONObject) item;
      pointTowardsZones.add(
          new PointTowardsZone(
              (String) zone.get("name"),
              translation((JSONObject) zone.get("fieldPosition")),
              Rotation2d.fromDegrees(number(zone.get("rotationOffset"))),
              number(zone.get("minWaypointRelativePos")),
              number(zone.get("maxWaypointRelativePos"))));
    }

    List<ConstraintsZone> constraintZones = new ArrayList<>();
    for (Object item : listOrEmpty(json.get("constraintZones"))) {
      JSONObject zone = (JSONObject) item;
      constraintZones.add(
          new ConstraintsZone(
              number(zone.get("minWaypointRelativePos")),
              number(zone.get("maxWaypointRelativePos")),
              constraintsFromJson((JSONObject) zone.get("constraints"))));
    }

    List<EventMarker> eventMarkers = new ArrayList<>();
    for (Object item : listOrEmpty(json.get("eventMarkers"))) {
      eventMarkers.add(markerFromJson((JSONObject) item));
    }

    PathConstraints globalConstraints =
        constraintsFromJson((JSONObject) json.get("globalConstraints"));

    IdealStartingState startingState = null;
    JSONObject start = (JSONObject) json.get("idealStartingState");
    if (start != null) {
      startingState =
          new IdealStartingState(
              number(start.get("velocity")), Rotation2d.fromDegrees(number(start.get("rotation"))));
    }

    JSONObject end = (JSONObject) json.get("goalEndState");
    GoalEndState goal =
        new GoalEndState(
            number(end.get("velocity")), Rotation2d.fromDegrees(number(end.get("rotation"))));

    boolean reversed = Boolean.TRUE.equals(json.get("reversed"));

    return new PathPlannerPath(
        waypoints,
        rotationTargets,
        pointTowardsZones,
        constraintZones,
        eventMarkers,
        globalConstraints,
        startingState,
        goal,
        reversed);
  }

  private static Waypoint waypointFromJson(JSONObject json) {
    Translation2d anchor = translation((JSONObject) json.get("anchor"));
    Translation2d prevControl = null;
    Translation2d nextControl = null;
    if (json.get("prevControl") != null) {
      prevControl = translation((JSONObject) json.get("prevControl"));
    }
    if (json.get("nextControl") != null) {
      nextControl = translation((JSONObject) json.get("nextControl"));
    }
    return new Waypoint(prevControl, anchor, nextControl);
  }

  private static EventMarker markerFromJson(JSONObject json) {
    String name = (String) json.get("name");
    double pos = number(json.get("waypointRelativePos"));
    double endPos = -1.0;
    if (json.get("endWaypointRelativePos") != null) {
      endPos = number(json.get("endWaypointRelativePos"));
    }
    Command command = null;
    if (json.get("command") != null) {
      try {
        command = CommandUtil.commandFromJson((JSONObject) json.get("command"), false, false);
      } catch (Exception e) {
        command = null; // like PathPlannerLib, a marker that can't load its command has none
      }
    }
    return new EventMarker(name, pos, endPos, command);
  }

  private static PathConstraints constraintsFromJson(JSONObject json) {
    // Angular limits are saved in degrees
    return new PathConstraints(
        number(json.get("maxVelocity")),
        number(json.get("maxAcceleration")),
        Math.toRadians(number(json.get("maxAngularVelocity"))),
        Math.toRadians(number(json.get("maxAngularAcceleration"))),
        number(json.get("nominalVoltage")),
        Boolean.TRUE.equals(json.get("unlimited")));
  }

  static Translation2d translation(JSONObject json) {
    return new Translation2d(number(json.get("x")), number(json.get("y")));
  }

  static double number(Object value) {
    return ((Number) value).doubleValue();
  }

  static JSONArray listOrEmpty(Object value) {
    if (value instanceof JSONArray) {
      return (JSONArray) value;
    }
    return new JSONArray();
  }
}
