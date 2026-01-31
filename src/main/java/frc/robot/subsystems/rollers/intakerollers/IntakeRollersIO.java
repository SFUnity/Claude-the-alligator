package frc.robot.subsystems.rollers.intakerollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {
  @AutoLog
  public static class IntakeRollersIOInputs {
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double velocity = 0.0;
  }

  public default void updateInputs(IntakeRollersIOInputs inputs) {}

  public default void runVolts(double volts) {}

  public default void stop() {}
}
