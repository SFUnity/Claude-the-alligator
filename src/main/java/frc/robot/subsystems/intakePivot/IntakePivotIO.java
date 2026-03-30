package frc.robot.subsystems.intakePivot;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
  @AutoLog
  public static class IntakePivotIOInputs {
    public double currentPositionDeg = 0;
    public double appliedVolts = 0.0;
    public double statorCurrent = 0.0;
    public double supplyCurrent = 0.0;
  }

  default void updateInputs(IntakePivotIOInputs inputs) {}

  default void runVolts(double volts) {}

  default void setPivotPosition(double setpointDeg) {}

  default void resetEncoder(double position) {}
}
