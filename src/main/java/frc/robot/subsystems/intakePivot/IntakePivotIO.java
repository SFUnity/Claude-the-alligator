package frc.robot.subsystems.intakePivot;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double pivotCurrentPositionDeg = 0;
    public double pivotAppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;

    public double rollersAppliedVolts = 0.0;
    public double rollersCurrentAmps = 0.0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void runRollers(double volts) {}

  default void runPivot(double volts) {}

  default void setPivotPosition(double setpointDeg) {}

  default void resetEncoder(double position) {}
}
