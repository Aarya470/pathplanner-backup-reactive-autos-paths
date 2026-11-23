package com.mercury1089.reactivepathing;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.junit.jupiter.api.Test;

public class RegionTest extends WpilibTest {
  /** A path the app saved, with a divert region and a game piece region with a zone */
  static JSONObject complexPath() throws Exception {
    InputStream in = RegionTest.class.getResourceAsStream("/Complex.path");
    String text = new String(in.readAllBytes(), StandardCharsets.UTF_8);
    return (JSONObject) new JSONParser().parse(text);
  }

  @Test
  public void readsRegionsTheAppSaved() throws Exception {
    List<Region> regions = Region.fromJson(complexPath());
    assertEquals(2, regions.size());

    Region divert = regions.get(0);
    assertEquals(Region.Trigger.ROBOT, divert.trigger);
    assertEquals(Region.Action.DIVERT, divert.action);
    assertEquals(0.2, divert.startPos, 1e-9);
    assertEquals(0.5, divert.endPos, 1e-9);
    assertEquals(1.0, divert.waitSeconds, 1e-9);
    PathPlannerPath divertPath = divert.divertPath;
    assertNotNull(divertPath);
    assertTrue(divertPath.getAllPathPoints().size() > 2);
    // The file saves turning limits in degrees; PathPlanner uses radians
    assertTrue(divertPath.getGlobalConstraints().maxAngularVelocityRadPerSec() < 20);

    Region piece = regions.get(1);
    assertEquals(Region.Trigger.GAME_PIECE, piece.trigger);
    assertEquals(4, piece.pickupZone.size());
    assertNull(piece.divertPath);

  }

  @Test
  public void aFileWithNoRegionsHasNone() throws Exception {
    JSONObject json = complexPath();
    json.remove("obstacleRegions");
    assertTrue(Region.fromJson(json).isEmpty());
  }

  @Test
  public void readsTheNamesTheAppSaves() throws Exception {
    JSONObject json = complexPath();
    JSONObject region = (JSONObject) ((JSONArray) json.get("obstacleRegions")).get(0);
    region.put("obstacleTeam", "opponent");
    assertEquals(Region.Team.OPPONENT, Region.fromJson(json).get(0).team);
    region.put("obstacleTeam", "teammate");
    assertEquals(Region.Team.TEAMMATE, Region.fromJson(json).get(0).team);
    region.put("obstacleTeam", "any");
    assertEquals(Region.Team.ANY, Region.fromJson(json).get(0).team);
    region.put("action", "continuePath");
    assertEquals(Region.Action.CONTINUE, Region.fromJson(json).get(0).action);
  }

  @Test
  public void tellsWhenAnAutoMirroredThePath() throws Exception {
    PathPlannerPath path = PathToolsTest.curvyPath();
    JSONObject inFile = fileWithWaypoints(path);
    assertFalse(Region.isMirrored(path, inFile));
    assertTrue(Region.isMirrored(path.mirrorPath(), inFile));
  }

  @Test
  public void aPathThatOnlyStartsOnTheMirrorPointIsNotMirrored() {
    // Starts on the middle line (same either way), but the rest of it is not mirrored
    double middleY = FlippingUtil.fieldSizeY / 2;
    PathPlannerPath path =
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                new Pose2d(2, middleY, Rotation2d.kZero), new Pose2d(5, 6, Rotation2d.kZero)),
            new PathConstraints(2, 2, 6, 6),
            null,
            new GoalEndState(0, Rotation2d.kZero));
    PathPlannerPath startsOnTheLineOnly =
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                new Pose2d(2, middleY, Rotation2d.kZero), new Pose2d(5, 2, Rotation2d.kZero)),
            new PathConstraints(2, 2, 6, 6),
            null,
            new GoalEndState(0, Rotation2d.kZero));
    assertFalse(Region.isMirrored(startsOnTheLineOnly, fileWithWaypoints(path)));
  }

  @Test
  public void mirroringMovesZonesAcrossTheField() throws Exception {
    List<Region> regions = Region.fromJson(complexPath());
    List<Region> mirrored = Region.mirror(regions);
    Translation2d corner = regions.get(1).pickupZone.get(0);
    Translation2d mirroredCorner = mirrored.get(1).pickupZone.get(0);
    assertEquals(corner.getX(), mirroredCorner.getX(), 1e-9);
    assertEquals(FlippingUtil.fieldSizeY - corner.getY(), mirroredCorner.getY(), 1e-9);
    assertNotNull(mirrored.get(0).divertPath);
  }

  @Test
  public void pointInsideAZone() {
    List<Translation2d> square =
        List.of(new Translation2d(0, 0), new Translation2d(2, 0), new Translation2d(2, 2), new Translation2d(0, 2));
    assertTrue(PathTools.isInside(square, new Translation2d(1, 1)));
    assertFalse(PathTools.isInside(square, new Translation2d(3, 1)));
  }

  /** Path file data with the waypoints of this path */
  @SuppressWarnings("unchecked")
  static JSONObject fileWithWaypoints(PathPlannerPath path) {
    JSONArray waypoints = new JSONArray();
    for (var waypoint : path.getWaypoints()) {
      JSONObject anchor = new JSONObject();
      anchor.put("x", waypoint.anchor().getX());
      anchor.put("y", waypoint.anchor().getY());
      JSONObject json = new JSONObject();
      json.put("anchor", anchor);
      waypoints.add(json);
    }
    JSONObject file = new JSONObject();
    file.put("waypoints", waypoints);
    return file;
  }
}
