package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface climbIO {

  @AutoLog
  public static class climbIOInputs {
    public double velocityRotsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  public default void updateInputs(climbIOInputs inputs) {}

  public default void runVolts(double voltage) {}
}
