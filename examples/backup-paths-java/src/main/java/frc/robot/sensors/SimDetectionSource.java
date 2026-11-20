package frc.robot.sensors;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.obstacle.AllianceColor;
import com.pathplanner.lib.obstacle.GamePiece;
import com.pathplanner.lib.obstacle.Obstacle;
import com.pathplanner.lib.obstacle.Point;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * SIMULATION ONLY. Replaces the detection camera with a fake robot and a fake game piece you place
 * from the dashboard. RobotContainer only creates this in the simulator.
 *
 * <ul>
 *   <li>Sim/Fake Robot: Off, Opponent, or Teammate. Sim/Fake Robot X and Y: where it sits.
 *   <li>Sim/Fake Piece: Off or On. Sim/Fake Piece X and Y: where it sits. It disappears once the
 *       robot drives over it, like it was picked up. Turn it Off and On to put it back.
 * </ul>
 *
 * <p>Positions are in meters, as drawn in the PathPlanner editor (blue side). They are flipped
 * for you when the sim is on the red alliance.
 */
public class SimDetectionSource implements DetectionSource {
    private static final String OFF = "Off";
    private static final String ON = "On";
    private static final String OPPONENT = "Opponent";
    private static final String TEAMMATE = "Teammate";
    private static final double PICKED_UP_WITHIN_M = 0.3;

    private final SendableChooser<String> robotChooser = new SendableChooser<>();
    private final SendableChooser<String> pieceChooser = new SendableChooser<>();
    private final Supplier<Pose2d> poseSupplier;
    private boolean pieceCollected = false;

    /** @param poseSupplier the robot pose, to tell when the fake piece has been driven over */
    public SimDetectionSource(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;

        robotChooser.setDefaultOption(OFF, OFF);
        robotChooser.addOption(OPPONENT, OPPONENT);
        robotChooser.addOption(TEAMMATE, TEAMMATE);
        SmartDashboard.putData("Sim/Fake Robot", robotChooser);
        SmartDashboard.putNumber("Sim/Fake Robot X", 1.9);
        SmartDashboard.putNumber("Sim/Fake Robot Y", 2.45);

        pieceChooser.setDefaultOption(OFF, OFF);
        pieceChooser.addOption(ON, ON);
        pieceChooser.onChange(choice -> pieceCollected = false);
        SmartDashboard.putData("Sim/Fake Piece", pieceChooser);
        SmartDashboard.putNumber("Sim/Fake Piece X", 2.0);
        SmartDashboard.putNumber("Sim/Fake Piece Y", 3.0);
    }

    @Override
    public List<Obstacle> getObstacles() {
        String mode = robotChooser.getSelected();
        if (mode == null || mode.equals(OFF)) {
            return List.of();
        }
        Translation2d spot = place("Sim/Fake Robot");
        boolean weAreRed = isRed();
        // Opponent wears the other alliance's color, teammate wears ours
        boolean wearsRed = mode.equals(OPPONENT) ? !weAreRed : weAreRed;
        AllianceColor color = wearsRed ? AllianceColor.RED : AllianceColor.BLUE;
        return List.of(new Obstacle(new Point(spot.getX(), spot.getY()), color, Timer.getFPGATimestamp()));
    }

    @Override
    public List<GamePiece> getGamePieces() {
        if (!ON.equals(pieceChooser.getSelected())) {
            return List.of();
        }
        Translation2d spot = place("Sim/Fake Piece");
        if (poseSupplier.get().getTranslation().getDistance(spot) < PICKED_UP_WITHIN_M) {
            pieceCollected = true;
        }
        if (pieceCollected) {
            return List.of();
        }
        return List.of(new GamePiece(new Point(spot.getX(), spot.getY()), Timer.getFPGATimestamp()));
    }

    private Translation2d place(String key) {
        Translation2d spot = new Translation2d(
            SmartDashboard.getNumber(key + " X", 0.0),
            SmartDashboard.getNumber(key + " Y", 0.0));
        return isRed() ? FlippingUtil.flipFieldPosition(spot) : spot;
    }

    private static boolean isRed() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    }
}
