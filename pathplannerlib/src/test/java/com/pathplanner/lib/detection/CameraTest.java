package com.pathplanner.lib.detection;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.obstacle.AllianceColor;
import com.pathplanner.lib.obstacle.FloorProjector;
import com.pathplanner.lib.obstacle.Point;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

public class CameraTest {
  private static final double EPS = 1e-6;

  /** A camera whose frames are given by the test, so no robot hardware is needed */
  private static final class TestCamera extends Camera {
    @Override
    protected Reading read() {
      return null;
    }
  }

  private static Camera.Reading angles(CameraTarget... targets) {
    return new Camera.Reading(List.of(targets), List.of(), 1.0, null);
  }

  @Test
  public void levelCameraSeesTargetStraightAhead() {
    // Camera 1 m up, tilted 45 degrees down, target on the floor in the middle of the image
    Optional<Translation2d> p = Camera.toRobotRelative(0, 0, 1.0, 45, 0, 0, 0, 0);
    assertTrue(p.isPresent());
    assertEquals(1.0, p.get().getX(), EPS);
    assertEquals(0.0, p.get().getY(), EPS);
  }

  @Test
  public void tyAboveCenterLooksFarther() {
    // 30 degrees down plus 10 up in the image = 20 degrees below level
    Translation2d p = Camera.toRobotRelative(0, 0, 1.0, 30, 0, 0, 0, 10).orElseThrow();
    assertEquals(1.0 / Math.tan(Math.toRadians(20)), p.getX(), EPS);
  }

  @Test
  public void txRightIsToTheRight() {
    Translation2d p = Camera.toRobotRelative(0, 0, 1.0, 45, 0, 0, 20, 0).orElseThrow();
    assertTrue(p.getY() < 0, "positive tx should be to the robot's right");
    assertTrue(p.getX() > 0);
  }

  @Test
  public void backwardCameraOffsetAndFacing() {
    // Camera 0.3 m behind the center, facing backward
    Translation2d p = Camera.toRobotRelative(-0.3, 0, 1.0, 45, 180, 0, 0, 0).orElseThrow();
    assertEquals(-1.3, p.getX(), EPS);
    assertEquals(0.0, p.getY(), EPS);
  }

  @Test
  public void targetHeightShortensDistance() {
    // Target middle half a meter up: the ray reaches it after half the drop
    Translation2d p = Camera.toRobotRelative(0, 0, 1.0, 45, 0, 0.5, 0, 0).orElseThrow();
    assertEquals(0.5, p.getX(), EPS);
  }

  @Test
  public void aboveTheHorizonIsDropped() {
    assertTrue(Camera.toRobotRelative(0, 0, 1.0, 0, 0, 0, 0, 5).isEmpty());
    assertTrue(Camera.toRobotRelative(0, 0, 1.0, 10, 0, 0, 0, 10).isEmpty());
  }

  @Test
  public void sightingsUseTheRobotPose() {
    Camera cam = new TestCamera().mountedAt(0, 0, 1.0, 45, 0);
    Pose2d robot = new Pose2d(5, 3, Rotation2d.fromDegrees(90));
    List<Sighting> s = cam.place(angles(new CameraTarget(0, 0, 0)), robot);
    assertEquals(1, s.size());
    // 1 m ahead of a robot facing +y
    assertEquals(5.0, s.get(0).fieldPosition().getX(), EPS);
    assertEquals(4.0, s.get(0).fieldPosition().getY(), EPS);
    assertEquals(1.0, s.get(0).distanceMeters(), EPS);
    assertEquals("OK", cam.getStatusWithoutPolling());
  }

  @Test
  public void classFilterAndColors() {
    Camera cam =
        new TestCamera().mountedAt(0, 0, 1.0, 45, 0).onlyClasses(1, 2).redClasses(1).blueClasses(2);
    List<Sighting> s =
        cam.place(
            angles(new CameraTarget(1, 0, 0), new CameraTarget(2, 5, 0), new CameraTarget(3, 0, 0)),
            new Pose2d());
    assertEquals(2, s.size());
    assertEquals(AllianceColor.RED, s.get(0).color());
    assertEquals(AllianceColor.BLUE, s.get(1).color());
  }

  @Test
  public void wrongClassIsReported() {
    Camera cam = new TestCamera().mountedAt(0, 0, 1.0, 45, 0).onlyClasses(0);
    assertTrue(cam.place(angles(new CameraTarget(4, 0, 0)), new Pose2d()).isEmpty());
    assertEquals("Wrong class ID 4", cam.getStatusWithoutPolling());
  }

  @Test
  public void maxRangeDropsFarTargets() {
    Camera cam = new TestCamera().mountedAt(0, 0, 1.0, 45, 0).maxRange(0.5);
    assertTrue(cam.place(angles(new CameraTarget(0, 0, 0)), new Pose2d()).isEmpty());
  }

  @Test
  public void defaultTargetHeightIsUsedUnlessSet() {
    Camera cam = new TestCamera().mountedAt(0, 0, 1.0, 45, 0);
    cam.setDefaultTargetHeight(0.5);
    assertEquals(
        0.5, cam.place(angles(new CameraTarget(0, 0, 0)), new Pose2d()).get(0).distanceMeters(), EPS);
    cam.targetHeight(0.0);
    assertEquals(
        1.0, cam.place(angles(new CameraTarget(0, 0, 0)), new Pose2d()).get(0).distanceMeters(), EPS);
  }

  @Test
  public void fieldPositionsPassThrough() {
    Camera cam = new TestCamera();
    List<Sighting> s =
        cam.place(
            new Camera.Reading(
                List.of(), List.of(new Camera.Placed(new Translation2d(2, 1), -1)), 1.0, null),
            new Pose2d(2, 0, new Rotation2d()));
    assertEquals(1, s.size());
    assertEquals(1.0, s.get(0).distanceMeters(), EPS);
    assertEquals(AllianceColor.UNKNOWN, s.get(0).color());
  }

  @Test
  public void limelightRawDetectionsParse() {
    double[] raw = {
      1, 4.5, -3.0, 0.2, 0, 0, 0, 0, 0, 0, 0, 0,
      2, -10.0, 1.5, 0.1, 0, 0, 0, 0, 0, 0, 0, 0,
      7, 1, 2 // a cut-off entry is ignored
    };
    List<CameraTarget> t = LimelightCamera.parse(raw);
    assertEquals(2, t.size());
    assertEquals(new CameraTarget(1, 4.5, -3.0), t.get(0));
    assertEquals(new CameraTarget(2, -10.0, 1.5), t.get(1));
    assertTrue(LimelightCamera.parse(new double[0]).isEmpty());
    assertTrue(LimelightCamera.parse(null).isEmpty());
  }

  @Test
  public void redAndBlueClassesAlsoFilter() {
    Camera cam = new TestCamera().mountedAt(0, 0, 1.0, 45, 0).redClasses(1).blueClasses(2);
    List<Sighting> s =
        cam.place(angles(new CameraTarget(0, 0, 0), new CameraTarget(2, 0, 0)), new Pose2d());
    assertEquals(1, s.size());
    assertEquals(2, s.get(0).classId());
  }

  @Test
  public void cameraBelowTargetLooksUp() {
    // A low camera sees the middle of a robot above it
    Camera cam = new TestCamera().mountedAt(0, 0, 0.13, 0, 0).targetHeight(0.4);
    List<Sighting> s =
        cam.place(angles(new CameraTarget(0, 0, Math.toDegrees(Math.atan(0.27 / 2.0)))), new Pose2d());
    assertEquals(1, s.size());
    assertEquals(2.0, s.get(0).distanceMeters(), 1e-6);
  }

  @Test
  public void statusExplainsMissedTargets() {
    Camera low = new TestCamera().mountedAt(0, 0, 0.3, 10, 0).targetHeight(0.4);
    assertTrue(low.place(angles(new CameraTarget(0, 0, 0)), new Pose2d()).isEmpty());
    assertTrue(low.getStatusWithoutPolling().startsWith("Ray never reaches"));

    Camera level = new TestCamera().mountedAt(0, 0, 0.4, 10, 0).targetHeight(0.4);
    assertTrue(level.place(angles(new CameraTarget(0, 0, 0)), new Pose2d()).isEmpty());
    assertTrue(level.getStatusWithoutPolling().startsWith("Camera is level with targetHeight"));
  }

  /** A floor point seen by a pinhole camera, for checking the projections */
  private static double[] pixelOf(
      double px, double py, double fwd, double left, double h, double tiltDeg, double faceDeg,
      double f, double cu, double cv) {
    double dx = px - fwd, dy = py - left, dz = -h;
    double yaw = Math.toRadians(faceDeg);
    double ax = dx * Math.cos(yaw) + dy * Math.sin(yaw);
    double ay = -dx * Math.sin(yaw) + dy * Math.cos(yaw);
    double p = Math.toRadians(tiltDeg);
    // Undo the downward tilt
    double cxv = ax * Math.cos(p) - dz * Math.sin(p);
    double czv = ax * Math.sin(p) + dz * Math.cos(p);
    return new double[] {cu - f * ay / cxv, cv - f * czv / cxv};
  }

  @Test
  public void floorProjectorIsExactOffCenter() {
    FloorProjector fp =
        new FloorProjector(0.75, Math.toRadians(15), Math.toRadians(30), 0.3, 0.1, 500, 500, 320, 240);
    for (double[] pt : new double[][] {{3.0, 0.5}, {4.0, -1.0}, {2.0, 2.0}}) {
      double[] uv = pixelOf(pt[0], pt[1], 0.3, 0.1, 0.75, 15, 30, 500, 320, 240);
      Point got = fp.projectToField(uv[0], uv[1], 0, 0, 0);
      assertEquals(pt[0], got.x(), 1e-6);
      assertEquals(pt[1], got.y(), 1e-6);
    }
  }

  @Test
  public void photonPitchConvertsToImagePlaneTy() {
    // A target 3 m ahead and 1.2 m right, 0.35 m below a level camera
    double nx = -1.2 / 3.0;
    double ny = -0.35 / 3.0;
    double yaw = Math.toDegrees(Math.atan(-nx)); // positive right
    // PhotonVision: pitch = atan(ny / (1 / cos(yaw))) = atan(ny * cos(yaw))
    double pvPitch = Math.toDegrees(Math.atan(ny * Math.cos(Math.toRadians(yaw))));
    double ty = PhotonVisionCamera.pitchToTy(yaw, pvPitch);
    assertEquals(Math.toDegrees(Math.atan(ny)), ty, 1e-9);
    Translation2d p = Camera.toRobotRelative(0, 0, 0.35, 0, 0, 0, yaw, ty).orElseThrow();
    assertEquals(3.0, p.getX(), 1e-6);
    assertEquals(-1.2, p.getY(), 1e-6);
  }
}
