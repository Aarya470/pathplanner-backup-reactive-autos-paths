// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.detection.PhotonGamePieceDetector;
import com.pathplanner.lib.detection.PhotonRobotDetector;
import com.pathplanner.lib.obstacle.GamePieceSettings;
import com.pathplanner.lib.obstacle.ObstacleSettings;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Detection;
import frc.robot.Constants.GamePieceDetection;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private PhotonRobotDetector detector;
  private PhotonGamePieceDetector pieceDetector;

  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Register named commands
    NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
    NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
    NamedCommands.registerCommand("print hello", Commands.print("hello"));

    // Use event markers as triggers
    new EventTrigger("Example Marker").onTrue(Commands.print("Passed an event marker"));

    // Robot and game piece detection for path regions. Must run before the autos are built.
    configureDetection();
    configurePickups();

    // Configure the trigger bindings
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  /**
   * Connect the robot detection camera. Any path with "robot in the way" regions in the PathPlanner
   * Backup Paths editor then waits or diverts when a robot blocks it, and the detector can be used
   * anywhere else in robot code too.
   */
  private void configureDetection() {
    int[] allRobots = new int[Detection.redRobotClassIds.length + Detection.blueRobotClassIds.length];
    System.arraycopy(Detection.redRobotClassIds, 0, allRobots, 0, Detection.redRobotClassIds.length);
    System.arraycopy(Detection.blueRobotClassIds, 0, allRobots, Detection.redRobotClassIds.length,
        Detection.blueRobotClassIds.length);

    detector = new PhotonRobotDetector(
        Detection.cameraName, Detection.robotToCamera, swerve::getPose, allRobots);
    // Lets regions set to "opponents" or "teammates" tell the robots apart
    detector.setAllianceClassIds(Detection.redRobotClassIds, Detection.blueRobotClassIds);
    ObstacleSettings.configure(detector, swerve::getPose, () -> swerve.driveRobotRelative(new ChassisSpeeds()));
  }

  /**
   * Connect the game piece camera and the intake. Any path with "game piece" regions (like
   * Pickup.path) then drives to a piece seen inside the region's orange zone, runs the intake, and
   * rejoins the path.
   */
  private void configurePickups() {
    pieceDetector = new PhotonGamePieceDetector(
        GamePieceDetection.cameraName, GamePieceDetection.robotToCamera, swerve::getPose,
        GamePieceDetection.pieceClassIds);
    GamePieceSettings.configure(
        pieceDetector,
        // Replace with your intake's command, for example intake::runIntakeCommand
        () -> Commands.print("Intake running").andThen(Commands.idle()),
        // Replace with your intake's beam break, for example intake::hasPiece. With null, the pickup
        // ends 0.3 s after reaching the piece.
        null,
        0.3, // seconds to keep the intake running after reaching the piece
        3.0, // give up on a pickup after this long
        GamePieceDetection.intakeFacingDeg);
        // Add your intake subsystem here as the last argument
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Robot detection outside of autos: print when a robot is within 1.5 m in front of the camera
    new Trigger(() -> detector.isRobotAhead(1.5)).onTrue(Commands.print("Robot ahead"));
    SmartDashboard.putData("Detection", builder -> {
      builder.addBooleanProperty("Robot Ahead", () -> detector.isRobotAhead(1.5), null);
      builder.addDoubleProperty("Closest Distance", () -> detector.getClosestRobot().isPresent()
          ? detector.getClosestRobot().get().getDistanceMeters() : -1, null);
      builder.addStringProperty("Status", detector::getStatus, null);
    });
    SmartDashboard.putData("Game Pieces", builder -> {
      builder.addDoubleProperty("Seen", () -> pieceDetector.getGamePieces().size(), null);
      builder.addStringProperty("Status", pieceDetector::getStatus, null);
    });

    // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser built above
    SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));

    // Add a button to run pathfinding commands to SmartDashboard
    SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
      new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0
    ));
    SmartDashboard.putData("Pathfind to Scoring Pos", AutoBuilder.pathfindToPose(
      new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)), 
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0
    ));

    // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // This example will simply move the robot 2m in the +X field direction
    SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
      Pose2d currentPose = swerve.getPose();
      
      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());

      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
        waypoints, 
        new PathConstraints(
          4.0, 4.0, 
          Units.degreesToRadians(360), Units.degreesToRadians(540)
        ),
        null, // Ideal starting state can be null for on-the-fly paths
        new GoalEndState(0.0, currentPose.getRotation())
      );

      // Prevent this path from being flipped on the red alliance, since the given positions are already correct
      path.preventFlipping = true;

      AutoBuilder.followPath(path).schedule();
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
