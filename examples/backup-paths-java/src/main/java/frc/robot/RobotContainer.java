// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.mercury1089.reactivepathing.ReactivePathing;
import com.mercury1089.detection.Camera;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

  // Object detection cameras, from Robot Detection.
  // mountedAt(forward m, left m, height m, tilt down degrees, facing degrees), measured from the
  // robot's center on the floor. Facing: 0 = forward, 180 = backward.
  //
  // Using a Limelight? Swap Camera.photonVision("...") for Camera.limelight("limelight-name").

  /** Sees robots. In the model, class 1 is a red robot and class 2 is a blue robot. */
  private final Camera robotCamera = Camera.photonVision("DetectionCam")
      .mountedAt(-0.41, 0.0, 0.13, 0, 180)
      .redRobotClasses(1)
      .blueRobotClasses(2);

  /** Sees game pieces. In the model, class 0 is a game piece. */
  private final Camera pieceCamera = Camera.photonVision("IntakeCam")
      .mountedAt(-0.36, 0.0, 0.30, 20, 180)
      .gamePieceClasses(0);

  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Register named commands
    NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
    NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
    NamedCommands.registerCommand("print hello", Commands.print("hello"));

    // Use event markers as triggers
    new EventTrigger("Example Marker").onTrue(Commands.print("Passed an event marker"));

    // Path regions: run this before the autos are built
    configureRegions();

    // Configure the trigger bindings
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  /**
   * Connect the cameras and intake to the regions drawn in the app. Paths with "robot in the way"
   * regions (like Score.path) then wait or divert when a robot blocks them, and paths with "game
   * piece" regions (like Pickup.path) pick up a piece seen in the orange zone and go back to the
   * path. Path following itself is set up in SwerveSubsystem, with ReactivePathing.configure(...).
   */
  private void configureRegions() {
    ReactivePathing.watchForRobots(robotCamera);

    ReactivePathing.intakeIsOnBack();
    ReactivePathing.pickUpGamePieces(
        pieceCamera,
        // Replace with your intake's command, for example intake::intakeCommand
        () -> Commands.print("Intake running").andThen(Commands.idle()),
        // Replace with your beam break, for example intake::hasPiece (null = run 0.5 s at the piece)
        null);
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
    // Cameras work outside of autos too: print when a robot is within 1.5 m
    robotCamera.robotNearby(1.5).onTrue(Commands.print("Robot nearby"));

    // Each camera's status ("OK", "Ignored class 3", ...) and what it sees, for setting up
    SmartDashboard.putData("Robot Camera", robotCamera);
    SmartDashboard.putData("Piece Camera", pieceCamera);

    // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser built above
    SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));

    // Add a button to run pathfinding commands to SmartDashboard. (With ReactivePathing, use its
    // pathfindToPose(...) instead of AutoBuilder's.)
    SmartDashboard.putData("Pathfind to Pickup Pos", ReactivePathing.pathfindToPose(
      new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0
    ));
    SmartDashboard.putData("Pathfind to Scoring Pos", ReactivePathing.pathfindToPose(
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

      CommandScheduler.getInstance().schedule(AutoBuilder.followPath(path));
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
