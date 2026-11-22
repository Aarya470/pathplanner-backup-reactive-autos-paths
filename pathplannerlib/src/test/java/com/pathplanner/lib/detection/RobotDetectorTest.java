package com.pathplanner.lib.detection;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.obstacle.AllianceColor;
import com.pathplanner.lib.obstacle.FloorProjector;
import com.pathplanner.lib.obstacle.Obstacle;
import com.pathplanner.lib.obstacle.Point;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

public class RobotDetectorTest {
  private static final double EPS = 1e-6;

  private static RobotDetector detectorSeeing(
      Pose2d robotPose, double cameraYawRad, Translation2d... others) {
    List<DetectedRobot> robots = new ArrayList<>();
    for (Translation2d other : others) {
      robots.add(new DetectedRobot(other, AllianceColor.UNKNOWN, 1.0, robotPose, cameraYawRad));
    }
    return new RobotDetector() {
      @Override
      public List<DetectedRobot> getRobots() {
        return robots;
      }

      @Override
      public double getAheadHalfAngleDegrees() {
        return 30.0;
      }

      @Override
      public double getMaxAgeSeconds() {
        return 0.25;
      }
    };
  }

  @Test
  public void distanceAndBearingFromTheRobotsFront() {
    Pose2d pose = new Pose2d(2, 2, Rotation2d.fromDegrees(90));
    DetectedRobot ahead =
        new DetectedRobot(new Translation2d(2, 4), AllianceColor.UNKNOWN, 1, pose, 0);
    assertEquals(2.0, ahead.getDistanceMeters(), EPS);
    assertEquals(0.0, ahead.getBearingDegrees(), EPS);

    DetectedRobot left =
        new DetectedRobot(new Translation2d(1, 2), AllianceColor.UNKNOWN, 1, pose, 0);
    assertEquals(90.0, left.getBearingDegrees(), EPS);
  }

  @Test
  public void cameraBearingUsesTheCameraDirection() {
    Pose2d pose = new Pose2d(5, 5, Rotation2d.kZero);
    DetectedRobot behind =
        new DetectedRobot(new Translation2d(3, 5), AllianceColor.UNKNOWN, 1, pose, Math.PI);
    assertEquals(180.0, Math.abs(behind.getBearingDegrees()), EPS);
    assertEquals(0.0, behind.getCameraBearingDegrees(), EPS);
  }

  @Test
  public void aheadMeansInFrontOfTheCameraAndCloseEnough() {
    Pose2d pose = new Pose2d(5, 5, Rotation2d.kZero);
    RobotDetector backCamera = detectorSeeing(pose, Math.PI, new Translation2d(3.8, 5.3));
    assertTrue(backCamera.isRobotAhead(1.5));
    assertFalse(backCamera.isRobotAhead(1.0));

    RobotDetector frontCamera = detectorSeeing(pose, 0, new Translation2d(3.8, 5.3));
    assertFalse(frontCamera.isRobotAhead(5));
    assertTrue(frontCamera.isRobotSeen());
  }

  @Test
  public void closestRobotWins() {
    Pose2d pose = new Pose2d(0, 0, Rotation2d.kZero);
    RobotDetector detector =
        detectorSeeing(pose, 0, new Translation2d(3, 0), new Translation2d(1, 0.2));
    assertEquals(1.0, detector.getClosestRobot().get().getFieldPosition().getX(), EPS);
    assertEquals(2, detector.getObstacles().size());
    Obstacle first = detector.getObstacles().get(0);
    assertEquals(3.0, first.position().x(), EPS);
  }

  @Test
  public void nothingSeenMeansNothingAhead() {
    RobotDetector detector = detectorSeeing(new Pose2d(), 0);
    assertFalse(detector.isRobotSeen());
    assertFalse(detector.isRobotAhead(10));
    assertTrue(detector.getClosestRobot().isEmpty());
  }

  @Test
  public void floorProjectionPutsABoxBottomAtTheRightDistance() {
    double height = 0.127;
    double fy = 600;
    double cy = 400;
    double distance = 2.0;
    double v = cy + fy * height / distance;
    FloorProjector projector = new FloorProjector(height, 0, Math.PI, -0.4, 0, 600, fy, 640, cy);
    Point spot = projector.projectToField(640, v, 5, 5, 0);
    assertEquals(5 - 0.4 - distance, spot.x(), 1e-3);
    assertEquals(5.0, spot.y(), 1e-3);
  }
}
