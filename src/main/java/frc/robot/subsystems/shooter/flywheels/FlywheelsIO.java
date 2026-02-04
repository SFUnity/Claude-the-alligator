package frc.robot.subsystems.shooter.flywheels;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelsIO {
  @AutoLog
  public static class FlywheelsIOInputs {
    public double appliedVolts = 0;
    public double velocityRotsPerSec = 0.0;
    public double statorCurrent = 0;
    public double supplyCurrent = 0;
  }

  public default void updateInputs(FlywheelsIOInputs inputs) {}

  public default void runVelocity(double rps) {}

  public default void ready() {}
}
