package com.mercury1089.reactivepathing;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.mercury1089.vision.Camera;
import com.mercury1089.vision.ThingType;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.util.FlippingUtil;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Runs whole autos on a simulated robot: real PathPlanner paths, followed by PathPlanner's real
 * path follower, with fake cameras that "see" robots and game pieces where the test puts them.
 */
public class SimulatedAutoTest extends WpilibTest {
  /** A perfect drivetrain: it moves exactly as it is told */
  static class SimDrive extends SubsystemBase {
    Pose2d pose = new Pose2d(2, 4, Rotation2d.kZero);
    ChassisSpeeds speeds = new ChassisSpeeds();

    void drive(ChassisSpeeds robotRelative) {
      speeds = robotRelative;
    }

    void step(double seconds) {
      pose =
          pose.exp(
              new Twist2d(
                  speeds.vxMetersPerSecond * seconds,
                  speeds.vyMetersPerSecond * seconds,
                  speeds.omegaRadiansPerSecond * seconds));
    }
  }

  static class SimIntake extends SubsystemBase {}

  SimDrive drive;
  SimIntake intake;
  List<Translation2d> robotsOnField;
  List<Translation2d> piecesOnField;
  boolean intakeRan;
  double closestToPiece;
  double timeStoppedInRegion;
  Translation2d regionEnd; // where timeStoppedInRegion is measured

  @BeforeEach
  public void setUp() {
    ReactivePathing.reset();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    SimHooks.pauseTiming();
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();

    drive = new SimDrive();
    intake = new SimIntake();
    robotsOnField = new ArrayList<>();
    piecesOnField = new ArrayList<>();
    intakeRan = false;
    closestToPiece = Double.POSITIVE_INFINITY;
    timeStoppedInRegion = 0;
    regionEnd = new Translation2d(5, 4);

    configure(false);
  }

  void configure(boolean onRed) {
    ModuleConfig module = new ModuleConfig(0.048, 4.5, 1.2, DCMotor.getKrakenX60(1).withReduction(6.14), 60, 1);
    RobotConfig config =
        new RobotConfig(
            50,
            6,
            module,
            new Translation2d(0.3, 0.3),
            new Translation2d(0.3, -0.3),
            new Translation2d(-0.3, 0.3),
            new Translation2d(-0.3, -0.3));
    ReactivePathing.configure(
        () -> drive.pose,
        (pose) -> drive.pose = pose,
        () -> drive.speeds,
        (speeds) -> drive.drive(speeds),
        new PPHolonomicDriveController(new PIDConstants(5, 0, 0), new PIDConstants(5, 0, 0)),
        config,
        () -> onRed,
        drive);
  }

  @AfterEach
  public void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    SimHooks.resumeTiming();
  }

  /** A straight path from (2, 4) to (8, 4). Position 0.5 is x = 5. */
  static PathPlannerPath straightPath() {
    PathPlannerPath path =
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                new Pose2d(2, 4, Rotation2d.kZero), new Pose2d(8, 4, Rotation2d.kZero)),
            new PathConstraints(2, 2, 6, 6),
            null,
            new GoalEndState(0, Rotation2d.kZero));
    path.name = "test";
    return path;
  }

  static PathPlannerPath divertPath() {
    return new PathPlannerPath(
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(5.5, 4.5, Rotation2d.fromDegrees(90)), new Pose2d(5.5, 7, Rotation2d.fromDegrees(90))),
        new PathConstraints(2, 2, 6, 6),
        null,
        new GoalEndState(0, Rotation2d.kZero));
  }

  static Region robotRegion(double waitSeconds, RegionAction action, RegionTeam team, PathPlannerPath divert) {
    return new Region(0.3, 0.5, RegionTrigger.ROBOT, team, waitSeconds, action, divert, List.of(), 1.5);
  }

  static Region pieceRegion(List<Translation2d> zone) {
    return new Region(
        0.2, 0.6, RegionTrigger.GAME_PIECE, RegionTeam.ANY, 0, RegionAction.CONTINUE, null, zone, 1.5);
  }

  /** A 1.6 m square around (4.5, 5) */
  static List<Translation2d> zoneAroundPiece() {
    return List.of(
        new Translation2d(3.7, 4.2),
        new Translation2d(5.3, 4.2),
        new Translation2d(5.3, 5.8),
        new Translation2d(3.7, 5.8));
  }

  void useCameras() {
    Camera robots = Camera.fromFieldPositions(ThingType.ROBOT, () -> robotsOnField);
    Camera pieces = Camera.fromFieldPositions(ThingType.GAME_PIECE, () -> piecesOnField);
    ReactivePathing.watchForRobots(robots);
    ReactivePathing.pickUpGamePieces(
        pieces,
        () -> Commands.run(() -> intakeRan = true, intake),
        () -> drive.pose.getTranslation().getDistance(new Translation2d(4.5, 5)) < 0.3,
        intake);
  }

  /** Run a command on the simulated robot. @return how long it took, in seconds */
  double run(Command command, double maxSeconds) throws InterruptedException {
    CommandScheduler.getInstance().schedule(command);
    double time = 0;
    while (time < maxSeconds) {
      drive.step(0.02);
      SimHooks.stepTiming(0.02);
      DriverStationSim.notifyNewData();
      DriverStation.refreshData(); // a real robot does this every loop
      CommandScheduler.getInstance().run();
      time += 0.02;

      if (!piecesOnField.isEmpty()) {
        closestToPiece =
            Math.min(closestToPiece, drive.pose.getTranslation().getDistance(piecesOnField.get(0)));
        if (closestToPiece < 0.3) {
          piecesOnField.clear(); // picked up
        }
      }
      boolean standingStill = Math.abs(drive.speeds.vxMetersPerSecond) < 0.01;
      boolean atRegionEnd = drive.pose.getTranslation().getDistance(regionEnd) < 0.3;
      if (standingStill && atRegionEnd) {
        timeStoppedInRegion += 0.02;
      }

      if (!command.isScheduled()) {
        return time;
      }
      Thread.sleep(1); // lets PathPlanner's pathfinder thread work
    }
    return time;
  }

  void assertEndsAt(double x, double y) {
    assertEquals(x, drive.pose.getX(), 0.25, "x at the end");
    assertEquals(y, drive.pose.getY(), 0.25, "y at the end");
  }

  // ------------------------------------------------------------------ tests

  @Test
  public void aPathWithNoRegionsIsAPlainPathPlannerPath() {
    Command command = ReactivePathing.commandForPath(straightPath());
    assertTrue(command instanceof FollowPathCommand);
  }

  @Test
  public void withNothingInTheWayItJustDrivesThePath() throws Exception {
    useCameras();
    double plainTime = run(ReactivePathing.followPathNormally(straightPath()), 20);
    drive.pose = new Pose2d(2, 4, Rotation2d.kZero);

    List<Region> regions = List.of(robotRegion(1.5, RegionAction.CONTINUE, RegionTeam.ANY, null));
    double time = run(new ReactivePathCommand(straightPath(), regions), 20);
    assertEndsAt(8, 4);
    assertEquals(plainTime, time, 0.2);
  }

  @Test
  public void waitsForARobotThenGoesOn() throws Exception {
    useCameras();
    robotsOnField.add(new Translation2d(4.2, 4.1)); // in the region, and it never moves

    List<Region> regions = List.of(robotRegion(1.5, RegionAction.CONTINUE, RegionTeam.ANY, null));
    run(new ReactivePathCommand(straightPath(), regions), 20);
    assertEndsAt(8, 4);
    assertTrue(timeStoppedInRegion >= 1.4, "stopped " + timeStoppedInRegion + " s at the region end");
  }

  @Test
  public void divertsIfTheRobotIsStillThereAfterWaiting() throws Exception {
    useCameras();
    robotsOnField.add(new Translation2d(4.2, 4.1));

    List<Region> regions = List.of(robotRegion(1.0, RegionAction.DIVERT, RegionTeam.ANY, divertPath()));
    run(new ReactivePathCommand(straightPath(), regions), 25);
    assertEndsAt(5.5, 7);
  }

  @Test
  public void goesOnInsteadOfDivertingIfTheRobotLeaves() throws Exception {
    useCameras();
    robotsOnField.add(new Translation2d(4.2, 4.1));
    List<Region> regions = List.of(robotRegion(1.0, RegionAction.DIVERT, RegionTeam.ANY, divertPath()));
    Command auto = new ReactivePathCommand(straightPath(), regions);

    CommandScheduler.getInstance().schedule(auto);
    // Drive until the robot has stopped at the region end, then the other robot leaves
    for (int i = 0; i < 400 && timeStoppedInRegion < 0.2; i++) {
      drive.step(0.02);
      SimHooks.stepTiming(0.02);
      DriverStationSim.notifyNewData();
      DriverStation.refreshData(); // a real robot does this every loop
      CommandScheduler.getInstance().run();
      boolean standingStill = Math.abs(drive.speeds.vxMetersPerSecond) < 0.01;
      if (standingStill && Math.abs(drive.pose.getX() - 5) < 0.3) {
        timeStoppedInRegion += 0.02;
      }
    }
    assertTrue(timeStoppedInRegion >= 0.2, "should have stopped at the region end");
    robotsOnField.clear();
    run(auto, 20);
    assertEndsAt(8, 4);
  }

  @Test
  public void opponentRegionsIgnoreRobotsOfUnknownColor() throws Exception {
    useCameras();
    robotsOnField.add(new Translation2d(4.2, 4.1)); // field positions have no color

    List<Region> regions = List.of(robotRegion(1.5, RegionAction.CONTINUE, RegionTeam.OPPONENT, null));
    run(new ReactivePathCommand(straightPath(), regions), 20);
    assertEndsAt(8, 4);
    assertTrue(timeStoppedInRegion < 0.2, "should not have stopped");
  }

  @Test
  public void picksUpAGamePieceInTheZoneThenFinishesThePath() throws Exception {
    useCameras();
    piecesOnField.add(new Translation2d(4.5, 5));

    run(new ReactivePathCommand(straightPath(), List.of(pieceRegion(zoneAroundPiece()))), 30);
    assertTrue(intakeRan, "the intake should have run");
    assertTrue(closestToPiece < 0.3, "drove to the piece, closest " + closestToPiece);
    assertEndsAt(8, 4);
  }

  @Test
  public void ignoresAGamePieceOutsideTheZone() throws Exception {
    useCameras();
    piecesOnField.add(new Translation2d(4.5, 7.5)); // far outside the zone

    run(new ReactivePathCommand(straightPath(), List.of(pieceRegion(zoneAroundPiece()))), 20);
    assertFalse(intakeRan);
    assertEndsAt(8, 4);
  }

  @Test
  public void switchesCanTurnPickupsOff() throws Exception {
    useCameras();
    ReactivePathing.setSwitches(() -> true, () -> true, () -> false);
    piecesOnField.add(new Translation2d(4.5, 5));

    run(new ReactivePathCommand(straightPath(), List.of(pieceRegion(zoneAroundPiece()))), 20);
    assertFalse(intakeRan);
    assertEndsAt(8, 4);
  }

  // ------------------------------------------------------------------ fixes from the reviews

  /**
   * Out to (5.5, 5.5) and back to just above the start. The robot starts a little off the start
   * point, closer to the end of the way back than to the start.
   */
  static PathPlannerPath outAndBackPath() {
    PathPlannerPath path =
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                new Pose2d(2, 4, Rotation2d.kZero),
                new Pose2d(5.5, 5.5, Rotation2d.fromDegrees(90)),
                new Pose2d(2, 4.1, Rotation2d.k180deg)),
            new PathConstraints(2, 2, 6, 6),
            null,
            new GoalEndState(0, Rotation2d.kZero));
    path.name = "test";
    return path;
  }

  @Test
  public void anOutAndBackPathDoesNotJumpToTheWayBack() throws Exception {
    useCameras();
    drive.pose = new Pose2d(2, 4.07, Rotation2d.kZero);
    piecesOnField.add(new Translation2d(4.5, 5));
    Region onTheWayOut =
        new Region(0.1, 0.9, RegionTrigger.GAME_PIECE, RegionTeam.ANY, 0, RegionAction.CONTINUE, null,
            zoneAroundPiece(), 2.0);

    run(new ReactivePathCommand(outAndBackPath(), List.of(onTheWayOut)), 40);
    assertTrue(intakeRan, "should have picked up the piece on the way out");
    assertEndsAt(2, 4.1);
  }

  @Test
  public void aRegionAtTheVeryEndOfThePathStillWorks() throws Exception {
    useCameras();
    robotsOnField.add(new Translation2d(7.8, 4.3)); // right by the end of the path
    PathPlannerPath divert =
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                new Pose2d(8, 4.5, Rotation2d.fromDegrees(90)), new Pose2d(8, 6.5, Rotation2d.fromDegrees(90))),
            new PathConstraints(2, 2, 6, 6),
            null,
            new GoalEndState(0, Rotation2d.kZero));
    Region atTheEnd =
        new Region(0.8, 1.0, RegionTrigger.ROBOT, RegionTeam.ANY, 0, RegionAction.DIVERT, divert, List.of(), 2.0);

    run(new ReactivePathCommand(straightPath(), List.of(atTheEnd)), 30);
    assertEndsAt(8, 6.5);
  }

  @Test
  public void anAutoCanRunTheIntakeAlongsideTheReactivePath() throws Exception {
    useCameras();
    piecesOnField.add(new Translation2d(4.5, 5));
    boolean[] autoIntakeRan = {false};
    Command path = new ReactivePathCommand(straightPath(), List.of(pieceRegion(zoneAroundPiece())));

    // Like a named command "intake" running in parallel with the path in an auto
    Command auto = Commands.deadline(path, Commands.run(() -> autoIntakeRan[0] = true, intake));

    run(auto, 30);
    assertTrue(autoIntakeRan[0], "the auto's own intake command ran");
    assertTrue(intakeRan, "ours ran too, run by the path command because the auto owns the intake");
    assertTrue(closestToPiece < 0.3, "drove to the piece, closest " + closestToPiece);
    assertEndsAt(8, 4);
  }

  @Test
  public void anIntakeMethodThatReturnsTheSameCommandEachTimeWorks() throws Exception {
    Command sharedIntake = Commands.run(() -> intakeRan = true, intake);
    ReactivePathing.watchForRobots(Camera.fromFieldPositions(ThingType.ROBOT, () -> robotsOnField));
    ReactivePathing.pickUpGamePieces(
        Camera.fromFieldPositions(ThingType.GAME_PIECE, () -> piecesOnField),
        () -> sharedIntake,
        () -> drive.pose.getTranslation().getDistance(new Translation2d(4.5, 5)) < 0.3,
        intake);

    for (int i = 0; i < 2; i++) {
      drive.pose = new Pose2d(2, 4, Rotation2d.kZero);
      piecesOnField.add(new Translation2d(4.5, 5));
      closestToPiece = Double.POSITIVE_INFINITY;
      intakeRan = false;
      run(new ReactivePathCommand(straightPath(), List.of(pieceRegion(zoneAroundPiece()))), 30);
      assertTrue(intakeRan, "the intake ran in run " + (i + 1));
      assertFalse(sharedIntake.isScheduled(), "the intake was stopped after the pickup");
      assertEndsAt(8, 4);
    }
  }

  @Test
  public void worksOnTheRedAlliance() throws Exception {
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    ReactivePathing.reset();
    configure(true);
    useCameras();

    drive.pose = new Pose2d(FlippingUtil.flipFieldPosition(new Translation2d(2, 4)), Rotation2d.k180deg);
    robotsOnField.add(FlippingUtil.flipFieldPosition(new Translation2d(4.2, 4.1)));
    regionEnd = FlippingUtil.flipFieldPosition(new Translation2d(5, 4));

    List<Region> regions = List.of(robotRegion(1.5, RegionAction.CONTINUE, RegionTeam.ANY, null));
    run(new ReactivePathCommand(straightPath(), regions), 20);
    Translation2d end = FlippingUtil.flipFieldPosition(new Translation2d(8, 4));
    assertEndsAt(end.getX(), end.getY());
    assertTrue(timeStoppedInRegion >= 1.4, "stopped " + timeStoppedInRegion + " s at the region end");
  }

  @Test
  public void canDivertAfterAPickup() throws Exception {
    useCameras();
    piecesOnField.add(new Translation2d(4.5, 5));
    Region pickupThenDivert =
        new Region(0.2, 0.6, RegionTrigger.GAME_PIECE, RegionTeam.ANY, 0, RegionAction.DIVERT,
            divertPath(), zoneAroundPiece(), 2.0);

    run(new ReactivePathCommand(straightPath(), List.of(pickupThenDivert)), 30);
    assertTrue(intakeRan);
    assertEndsAt(5.5, 7);
  }

  @Test
  public void mirroredRegionsMirrorTheirZonesAndDivertPaths() {
    Region region = pieceRegion(zoneAroundPiece());
    Region withDivert = robotRegion(0, RegionAction.DIVERT, RegionTeam.ANY, divertPath());
    List<Region> mirrored = ReactivePathing.mirrorRegions(List.of(region, withDivert));
    double fieldY = FlippingUtil.fieldSizeY;
    assertEquals(fieldY - 4.2, mirrored.get(0).getPickupZone().get(0).getY(), 1e-9);
    assertEquals(
        fieldY - 7, mirrored.get(1).getDivertPath().getWaypoints().get(1).anchor().getY(), 1e-9);
  }

  @Test
  public void theIntakeRunsWhenTheAutoUsesTheIntakeLater() throws Exception {
    useCameras();
    piecesOnField.add(new Translation2d(4.5, 5));
    Command path = new ReactivePathCommand(straightPath(), List.of(pieceRegion(zoneAroundPiece())));

    // The auto uses the intake after the path (like a "score" named command), so the auto
    // holds the intake the whole time
    Command auto = Commands.sequence(path, Commands.runOnce(() -> {}, intake));

    run(auto, 30);
    assertTrue(intakeRan, "the intake ran during the pickup");
    assertEndsAt(8, 4);
  }

  @Test
  public void aPickupNeverCancelsTheAuto() throws Exception {
    // The intake command also needs the drivetrain, and no intake subsystem was given
    ReactivePathing.watchForRobots(Camera.fromFieldPositions(ThingType.ROBOT, () -> robotsOnField));
    ReactivePathing.pickUpGamePieces(
        Camera.fromFieldPositions(ThingType.GAME_PIECE, () -> piecesOnField),
        () -> Commands.run(() -> intakeRan = true, intake, drive),
        () -> drive.pose.getTranslation().getDistance(new Translation2d(4.5, 5)) < 0.3);
    piecesOnField.add(new Translation2d(4.5, 5));

    Command auto = new ReactivePathCommand(straightPath(), List.of(pieceRegion(zoneAroundPiece())));
    run(auto, 30);
    assertEndsAt(8, 4);
  }
}
