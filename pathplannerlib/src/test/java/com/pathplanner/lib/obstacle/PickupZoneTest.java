package com.pathplanner.lib.obstacle;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.junit.jupiter.api.Test;

public class PickupZoneTest {
  private static final List<Point> SQUARE =
      List.of(new Point(1, 1), new Point(3, 1), new Point(3, 3), new Point(1, 3));

  @Test
  public void pieceInsideTheZoneCounts() {
    assertTrue(RegionGeometry.insidePolygon(SQUARE, 2, 2));
    assertTrue(RegionGeometry.insidePolygon(SQUARE, 1.01, 2.99));
  }

  @Test
  public void pieceOutsideTheZoneDoesNot() {
    assertFalse(RegionGeometry.insidePolygon(SQUARE, 0.5, 2));
    assertFalse(RegionGeometry.insidePolygon(SQUARE, 2, 3.5));
    assertFalse(RegionGeometry.insidePolygon(SQUARE, 10, 10));
  }

  @Test
  public void tiltedFourCornerZoneWorks() {
    List<Point> diamond =
        List.of(new Point(2, 0), new Point(4, 2), new Point(2, 4), new Point(0, 2));
    assertTrue(RegionGeometry.insidePolygon(diamond, 2, 2));
    assertFalse(RegionGeometry.insidePolygon(diamond, 0.3, 0.3));
  }

  @Test
  public void fewerThanThreeCornersIsNoZone() {
    assertFalse(RegionGeometry.insidePolygon(List.of(new Point(0, 0), new Point(1, 1)), 0.5, 0.5));
    assertFalse(RegionGeometry.insidePolygon(null, 0, 0));
  }

  @Test
  public void zoneIsReadFromThePathFile() throws Exception {
    String json =
        "{\"startPos\":0.2,\"endPos\":0.9,\"obstacleTeam\":\"any\",\"action\":\"continuePath\","
            + "\"trigger\":\"gamePiece\",\"pieceMaxDistance\":1.5,\"pickupZone\":["
            + "{\"x\":1.0,\"y\":1.0},{\"x\":3.0,\"y\":1.0},{\"x\":3.0,\"y\":3.0},{\"x\":1.0,\"y\":3.0}]}";
    ObstacleRegion region =
        ObstacleRegion.fromJson((JSONObject) new JSONParser().parse(json), j -> null);
    assertTrue(region.hasPickupZone());
    assertEquals(SQUARE, region.getPickupZone());
    assertEquals(1.5, region.getPieceMaxDistanceM(), 1e-9);
  }

  @Test
  public void oldPathFilesHaveNoZone() throws Exception {
    String json =
        "{\"startPos\":0.2,\"endPos\":0.9,\"obstacleTeam\":\"any\",\"action\":\"continuePath\","
            + "\"trigger\":\"gamePiece\",\"pieceMaxDistance\":1.5}";
    ObstacleRegion region =
        ObstacleRegion.fromJson((JSONObject) new JSONParser().parse(json), j -> null);
    assertFalse(region.hasPickupZone());
    assertTrue(region.getPickupZone().isEmpty());
  }

  @Test
  public void headingBlendsBetweenRotations() {
    RotationProfile profile =
        new RotationProfile(
            new double[] {0.0, 1.0, 2.0},
            new Rotation2d[] {
              Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90)
            });
    assertEquals(0.0, profile.at(0.0).getDegrees(), 1e-6);
    assertEquals(45.0, profile.at(0.5).getDegrees(), 1e-6);
    assertEquals(90.0, profile.at(1.5).getDegrees(), 1e-6);
    assertEquals(90.0, profile.at(5.0).getDegrees(), 1e-6);
  }

  @Test
  public void pathWithNoRotationsHasNoHeading() {
    assertNull(new RotationProfile(new double[0], new Rotation2d[0]).at(1.0));
  }
}
