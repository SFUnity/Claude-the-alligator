package frc.robot.subsystems.intakePivot;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
  @AutoLog
  public static class IntakePivotIOInputs {
    public double pivotCurrentPositionDeg = 0;
    public double pivotAppliedVolts = 0.0;
    public double pivotStaterCurrent = 0.0;
    public double pivotSupplyCurrent = 0.0;
  }

  default void updateInputs(IntakePivotIOInputs inputs) {}

  // TODO rename to runVolts
  default void runVolts(double volts) {}

  default void setPivotPosition(double setpointDeg) {}

  default void resetEncoder(double position) {}
}
