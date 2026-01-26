package frc.robot.subsystems.rollers.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double velocity = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void runVolts(double volts) {}

  public default void stop() {}
}
