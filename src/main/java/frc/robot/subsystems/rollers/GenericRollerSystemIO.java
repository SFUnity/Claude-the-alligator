package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface GenericRollerSystemIO {
  @AutoLog
  public static class GenericRollerInputs {
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(GenericRollerInputs inputs) {}

  public default void runVolts(double volts) {}

  public default void stop() {}
}
