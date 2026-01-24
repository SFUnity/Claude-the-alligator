package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public double positionDeg = 0;
    public double appliedVolts = 0;
    public double statorCurrent = 0;
    public double supplyCurrent = 0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setPosition(double positionDeg) {}
}
