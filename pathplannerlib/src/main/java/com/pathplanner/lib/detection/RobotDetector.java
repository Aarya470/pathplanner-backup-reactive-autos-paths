package com.pathplanner.lib.detection;

import com.pathplanner.lib.obstacle.Obstacle;
import com.pathplanner.lib.obstacle.ObstacleSource;
import com.pathplanner.lib.obstacle.Point;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Finds other robots on the field. Use it anywhere in robot code (autos, teleop, triggers), and
 * pass it to {@link com.pathplanner.lib.obstacle.ObstacleSettings#configure(RobotDetector,
 * java.util.function.Supplier, Runnable)} so path regions can react to it.
 *
 * <p>{@link PhotonRobotDetector} is ready to use with PhotonVision. For another camera, implement
 * {@link #getRobots()} and {@link #getAheadHalfAngleDegrees()}; everything else comes from those.
 */
public interface RobotDetector extends ObstacleSource {
  /** @return every robot seen recently, measured from your robot's current pose */
  List<DetectedRobot> getRobots();

  /**
   * @return how far to each side of the camera's direction still counts as "ahead", in degrees.
   *     30 means a 60 degree wide cone.
   */
  double getAheadHalfAngleDegrees();

  /** @return how old a detection may be before it is ignored, in seconds */
  double getMaxAgeSeconds();

  /** @return true if any robot is seen right now */
  default boolean isRobotSeen() {
    return !getRobots().isEmpty();
  }

  /** @return the closest robot seen, or empty if none */
  default Optional<DetectedRobot> getClosestRobot() {
    DetectedRobot closest = null;
    for (DetectedRobot robot : getRobots()) {
      if (closest == null || robot.getDistanceMeters() < closest.getDistanceMeters()) {
        closest = robot;
      }
    }
    return Optional.ofNullable(closest);
  }

  /** @return the closest robot in front of the camera, or empty if none */
  default Optional<DetectedRobot> getClosestRobotAhead() {
    DetectedRobot closest = null;
    for (DetectedRobot robot : getRobots()) {
      if (Math.abs(robot.getCameraBearingDegrees()) > getAheadHalfAngleDegrees()) {
        continue;
      }
      if (closest == null || robot.getDistanceMeters() < closest.getDistanceMeters()) {
        closest = robot;
      }
    }
    return Optional.ofNullable(closest);
  }

  /**
   * @param maxDistanceMeters how close the robot must be, measured from your robot's center
   * @return true if a robot is in front of the camera and within that distance
   */
  default boolean isRobotAhead(double maxDistanceMeters) {
    Optional<DetectedRobot> closest = getClosestRobotAhead();
    return closest.isPresent() && closest.get().getDistanceMeters() <= maxDistanceMeters;
  }

  /** Path regions read robots through this. */
  @Override
  default List<Obstacle> getObstacles() {
    List<Obstacle> obstacles = new ArrayList<>();
    for (DetectedRobot robot : getRobots()) {
      obstacles.add(
          new Obstacle(
              new Point(robot.getFieldPosition().getX(), robot.getFieldPosition().getY()),
              robot.getAlliance(),
              robot.getTimestampSeconds()));
    }
    return obstacles;
  }
}
