package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface GenericRollerIO {
  @AutoLog
  public static class GenericRollerIOInputs {
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(GenericRollerIOInputs inputs) {}

  public default void runVolts(double volts) {}

  public default void stop() {}
}
