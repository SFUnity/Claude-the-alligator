package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

  @AutoLog
  public static class ClimbIOInputs {
    public double velocityRotsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double positionMeters = 0.0;
  }

  public default void updateInputs(ClimbIOInputs inputs) {}

  // TODO make the units consistent. Right now you're constant is in meters
  public default void setPosition(double rotations) {}
}
