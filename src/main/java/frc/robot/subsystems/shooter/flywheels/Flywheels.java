package frc.robot.subsystems.shooter.flywheels;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  public Command setVelocity(double speed) {
    return run(() -> updateFlywheels(speed));
  }

  public Command setIdle() {
    return run(() -> idle = true);
  }
}
