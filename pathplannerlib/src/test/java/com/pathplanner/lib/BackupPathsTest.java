package com.pathplanner.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;

import com.pathplanner.lib.detection.Camera;
import com.pathplanner.lib.obstacle.GamePieceSettings;
import com.pathplanner.lib.obstacle.ObstacleSettings;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import java.util.function.Supplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

public class BackupPathsTest {
  @AfterEach
  public void cleanUp() {
    BackupPaths.reset();
  }

  @Test
  public void setupThenCamerasInAnyOrder() {
    Supplier<Pose2d> pose = Pose2d::new;
    Camera robots = Camera.fromAngles(List::of).maxAge(0.4);
    BackupPaths.watchForRobots(robots);
    BackupPaths.setup(pose, () -> {});

    assertSame(pose, ObstacleSettings.getPoseSupplier());
    assertSame(pose, BackupPaths.getPoseSupplier());
    assertSame(robots, ObstacleSettings.getSource());
    // The camera filters by its own maxAge, so nothing is dropped a second time
    assertEquals(Double.POSITIVE_INFINITY, ObstacleSettings.getStaleAfterSec());
  }

  @Test
  public void gamePiecesOnly() {
    BackupPaths.setup(Pose2d::new, null);
    Camera pieces = Camera.fromAngles(List::of);
    BackupPaths.intakeIsOnBack();
    BackupPaths.pickUpGamePieces(pieces, null, null);

    assertNull(ObstacleSettings.getSource());
    assertSame(pieces, BackupPaths.getPieceCamera());
    assertEquals(180, GamePieceSettings.getIntakeFacingDegrees(), 1e-9);
  }

  @Test
  public void oldApiKeepsItsStaleTimeForPieces() {
    ObstacleSettings.configure(null, Pose2d::new, () -> {}, 0.5);
    GamePieceSettings.configure(null, null, null, 0.5, 3.0, 0);
    assertEquals(0.5, GamePieceSettings.getStaleAfterSecForTests(), 1e-9);

    BackupPaths.setup(Pose2d::new, null);
    BackupPaths.pickUpGamePieces(Camera.fromAngles(List::of), null, null);
    assertEquals(Double.POSITIVE_INFINITY, GamePieceSettings.getStaleAfterSecForTests());
  }

  @Test
  public void cameraGivenToOldApiGetsDefaultTargetHeight() {
    Camera robots = Camera.fromAngles(List::of);
    ObstacleSettings.configure(robots, Pose2d::new, () -> {}, 0.25);
    assertEquals(BackupPaths.ROBOT_TARGET_HEIGHT, robots.getDefaultTargetHeightForTests(), 1e-9);
  }
}
