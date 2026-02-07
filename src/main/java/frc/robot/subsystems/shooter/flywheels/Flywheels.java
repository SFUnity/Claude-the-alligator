package frc.robot.subsystems.shooter.flywheels;

import static frc.robot.subsystems.shooter.flywheels.FlywheelsConstants.flywheelTolerance;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Flywheels extends SubsystemBase {
  private final FlywheelsIO io;
  private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();
  private double velocity;

  private boolean ready = false;

  public Flywheels(FlywheelsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheels", inputs);
    // TODO add logging from GeneralUtil

    if (!ready) {
      io.runVelocity(velocity);
    } else {
      io.ready();
    }
  }

  private void updateFlywheels(double speed) {
    ready = false;
    velocity = speed;
  }


  public Command setVelocity(double rpm) {
    return run(() -> updateFlywheels(rpm));
  }

  public Command setReady(boolean ready) {
    return run(() -> this.ready = ready);
  }

  public boolean atGoal() {
    return Math.abs(inputs.velocityRotsPerMin - velocity) < flywheelTolerance.get();
  }
}
