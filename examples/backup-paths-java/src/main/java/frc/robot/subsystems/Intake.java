package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A one-motor intake. Swap in your own intake subsystem and command. */
public class Intake extends SubsystemBase {
  private final PWMSparkMax motor = new PWMSparkMax(0);

  public Command intake() {
    return startEnd(() -> motor.set(0.8), () -> motor.set(0.0));
  }
}
