package com.pathplanner.lib.obstacle;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Setup for game piece regions. Call {@link #configure} once at robot startup. Without it, game
 * piece regions in path files are ignored and the robot follows the main path.
 */
public final class GamePieceSettings {
  private static GamePieceSource source = null;
  private static Supplier<Command> intakeCommand = Commands::none;
  private static BooleanSupplier hasPiece = null;
  private static double intakeAfterArriveSec = 0.5;
  private static double maxPickupSec = 3.0;
  private static double intakeFacingDeg = 0.0;
  private static Subsystem[] intakeRequirements = new Subsystem[0];
  private static BooleanSupplier enabled = () -> true;

  private GamePieceSettings() {}

  /**
   * @param pieceSource where game pieces on the field come from
   * @param intake makes the command that runs the intake while picking up. It is stopped when the
   *     pickup ends.
   * @param hasPieceSensor true while the robot is holding a piece (for example an intake beam
   *     break), or null if the robot has no such sensor. With one, a pickup ends as soon as it
   *     reads true, and pickups are skipped while it already reads true. Without one, the pickup
   *     ends after the robot reaches the piece and runs the intake for intakeAfterArriveSec.
   * @param intakeAfterArriveSec seconds to keep the intake running after reaching the piece
   * @param maxPickupSec give up on a pickup after this long
   * @param intakeFacingDeg which way the intake faces on the robot: 0 for the front, 180 for the
   *     back
   * @param intakeSubsystems the subsystems the intake command uses
   */
  public static void configure(
      GamePieceSource pieceSource,
      Supplier<Command> intake,
      BooleanSupplier hasPieceSensor,
      double intakeAfterArriveSec,
      double maxPickupSec,
      double intakeFacingDeg,
      Subsystem... intakeSubsystems) {
    source = pieceSource;
    intakeCommand = intake == null ? Commands::none : intake;
    hasPiece = hasPieceSensor;
    GamePieceSettings.intakeAfterArriveSec = intakeAfterArriveSec;
    GamePieceSettings.maxPickupSec = maxPickupSec;
    GamePieceSettings.intakeFacingDeg = intakeFacingDeg;
    intakeRequirements = intakeSubsystems;
  }

  /**
   * Dashboard control. When it returns false, game piece regions are skipped.
   *
   * @param pickupEnabled true to allow pickups
   */
  public static void setEnabled(BooleanSupplier pickupEnabled) {
    enabled = pickupEnabled == null ? () -> true : pickupEnabled;
  }

  static boolean isActive() {
    return source != null && enabled.getAsBoolean();
  }

  static GamePieceSource getSource() {
    return source;
  }

  static Command makeIntakeCommand() {
    return intakeCommand.get();
  }

  static BooleanSupplier getHasPiece() {
    return hasPiece;
  }

  static double getIntakeAfterArriveSec() {
    return intakeAfterArriveSec;
  }

  static double getMaxPickupSec() {
    return maxPickupSec;
  }

  static double getIntakeFacingDeg() {
    return intakeFacingDeg;
  }

  static Subsystem[] getIntakeRequirements() {
    return intakeRequirements;
  }
}
