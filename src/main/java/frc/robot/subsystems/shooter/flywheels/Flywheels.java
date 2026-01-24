package frc.robot.subsystems.shooter.flywheels;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheels extends SubsystemBase {
  private final FlywheelsIO topIO;
  private final FlywheelsIOInputsAutoLogged topInputs = new FlywheelsIOInputsAutoLogged();
  private final FlywheelsIO bottomIO;
  private final FlywheelsIOInputsAutoLogged bottomInputs = new FlywheelsIOInputsAutoLogged();

  private double topVelocity;
  private double bottomVelocity;

  private boolean idle = false;

  public Flywheels(FlywheelsIO topIO, FlywheelsIO bottomIO) {
    this.topIO = topIO;
    this.bottomIO = bottomIO;
  }

  public void periodic() {
    topIO.updateInputs(topInputs);
    bottomIO.updateInputs(bottomInputs);

    if (!idle) {
      topIO.runVelocity(topVelocity);
      bottomIO.runVelocity(bottomVelocity);
    } else {
      topIO.idle();
      bottomIO.idle();
    }
  }

  private void updateFlywheels(double top, double bottom) {
    idle = false;
    topVelocity = top;
    bottomVelocity = bottom;
  }

  public Command setVelocities(double top, double bottom) {
    return run(() -> updateFlywheels(top, bottom));
  }

  public Command setIdle() {
    return run(() -> idle = true);
  }
}
