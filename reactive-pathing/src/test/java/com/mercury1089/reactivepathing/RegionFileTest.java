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
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.junit.jupiter.api.Test;

public class RegionFileTest extends WpilibTest {
  /** A path the app saved, with a divert region and a game piece region with a zone */
  static JSONObject complexPath() throws Exception {
    InputStream in = RegionFileTest.class.getResourceAsStream("/Complex.path");
    String text = new String(in.readAllBytes(), StandardCharsets.UTF_8);
    return (JSONObject) new JSONParser().parse(text);
  }

  @Test
  public void readsRegionsTheAppSaved() throws Exception {
    RegionFile.PathFileData data = RegionFile.fromJson(complexPath());
    List<Region> regions = data.getRegions();
    assertEquals(2, regions.size());

    Region divert = regions.get(0);
    assertEquals(RegionTrigger.ROBOT, divert.getTrigger());
    assertEquals(RegionAction.DIVERT, divert.getAction());
    assertEquals(0.2, divert.getStartPos(), 1e-9);
    assertEquals(0.5, divert.getEndPos(), 1e-9);
    assertEquals(1.0, divert.getWaitSeconds(), 1e-9);
    PathPlannerPath divertPath = divert.getDivertPath();
    assertNotNull(divertPath);
    assertTrue(divertPath.getAllPathPoints().size() > 2);

    Region piece = regions.get(1);
    assertEquals(RegionTrigger.GAME_PIECE, piece.getTrigger());
    assertTrue(piece.hasPickupZone());
    assertEquals(4, piece.getPickupZone().size());
    assertNull(piece.getDivertPath());

    assertFalse(data.getWaypoints().isEmpty());
  }

  @Test
  public void aFileWithNoRegionsHasNone() throws Exception {
    JSONObject json = complexPath();
    json.remove("obstacleRegions");
    assertTrue(RegionFile.fromJson(json).getRegions().isEmpty());
  }

  @Test
  public void namesTheAppSaves() {
    assertEquals(RegionTrigger.GAME_PIECE, RegionTrigger.fromName("gamePiece"));
    assertEquals(RegionTrigger.ROBOT, RegionTrigger.fromName("robot"));
    assertEquals(RegionTrigger.ROBOT, RegionTrigger.fromName(null));
    assertEquals(RegionAction.DIVERT, RegionAction.fromName("divert"));
    assertEquals(RegionAction.CONTINUE, RegionAction.fromName("continuePath"));
    assertEquals(RegionTeam.OPPONENT, RegionTeam.fromName("opponent"));
    assertEquals(RegionTeam.TEAMMATE, RegionTeam.fromName("teammate"));
    assertEquals(RegionTeam.ANY, RegionTeam.fromName("any"));
  }

  @Test
  public void tellsWhenAnAutoMirroredThePath() throws Exception {
    PathPlannerPath path = PathCutterTest.curvyPath();
    List<Translation2d> inFile = new ArrayList<>();
    for (var waypoint : path.getWaypoints()) {
      inFile.add(waypoint.anchor());
    }
    assertFalse(ReactivePathing.isMirrored(path, inFile));
    PathPlannerPath mirrored = path.mirrorPath();
    assertTrue(ReactivePathing.isMirrored(mirrored, inFile));
    assertFalse(ReactivePathing.isMirrored(path, null));
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
    List<Translation2d> inFile =
        List.of(new Translation2d(2, middleY), new Translation2d(5, 6));
    assertFalse(ReactivePathing.isMirrored(path, inFile));
  }

  @Test
  public void mirroringMovesZonesAcrossTheField() throws Exception {
    List<Region> regions = RegionFile.fromJson(complexPath()).getRegions();
    List<Region> mirrored = ReactivePathing.mirrorRegions(regions);
    Translation2d corner = regions.get(1).getPickupZone().get(0);
    Translation2d mirroredCorner = mirrored.get(1).getPickupZone().get(0);
    assertEquals(corner.getX(), mirroredCorner.getX(), 1e-9);
    assertEquals(FlippingUtil.fieldSizeY - corner.getY(), mirroredCorner.getY(), 1e-9);
    assertNotNull(mirrored.get(0).getDivertPath());
  }

  @Test
  public void pointInsideAZone() {
    List<Translation2d> square =
        List.of(new Translation2d(0, 0), new Translation2d(2, 0), new Translation2d(2, 2), new Translation2d(0, 2));
    assertTrue(PathMath.isInside(square, new Translation2d(1, 1)));
    assertFalse(PathMath.isInside(square, new Translation2d(3, 1)));
  }
}
