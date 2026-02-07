package frc.robot.subsystems.rollers.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double velocity = 0.0;
  }

  public default void updateInputs(KickerIOInputs inputs) {}

  public default void runVolts(double volts) {}

  public default void stop() {}

  public default void runVelocity(double rps) {}
}
