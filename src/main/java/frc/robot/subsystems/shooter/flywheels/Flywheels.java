package frc.robot.subsystems.shooter.flywheels;

import static frc.robot.subsystems.shooter.flywheels.FlywheelsConstants.flywheelTolerance;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Flywheels extends SubsystemBase {
  private final FlywheelsIO io;
  private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();
  private double velocity;

  private boolean idle = false;

  public Flywheels(FlywheelsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheels", inputs);

    if (!idle) {
      io.runVelocity(velocity);
    } else {
      io.idle();
    }
  }

  private void updateFlywheels(double speed) {
    idle = false;
    velocity = speed;
  }

  public Command setVelocity(double rps) {
    return run(() -> updateFlywheels(rps));
  }

  public Command setIdle(boolean idle) {
    return run(() -> this.idle = idle);
  }

  public boolean atGoal() {
    return Math.abs(inputs.velocityRotsPerSec - velocity) < flywheelTolerance;
  }
}
