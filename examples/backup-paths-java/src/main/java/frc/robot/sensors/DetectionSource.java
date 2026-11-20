package frc.robot.sensors;

import com.pathplanner.lib.obstacle.GamePieceSource;
import com.pathplanner.lib.obstacle.ObstacleSource;

/** Something that reports both robots and game pieces: the detection camera or the sim fake. */
public interface DetectionSource extends ObstacleSource, GamePieceSource {}
