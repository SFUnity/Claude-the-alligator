package frc.robot.subsystems.shooter.flywheels;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelsIO {
  @AutoLog
  public static class FlywheelsIOInputs {
    public double appliedVolts = 0;

    public double velocityRotsPerMin = 0.0;
    public double statorCurrent = 0;
    public double supplyCurrent = 0;
  }

  public default void updateInputs(FlywheelsIOInputs inputs) {}

  public default void stop() {}

  public default void runVolts(double volts) {}

  public default void runDutyCycle() {}

  public default void runSlowDutyCycle() {}

  public default void runTorqueControl() {}

  public default void runTorqueControl(double rps) {}
}
