package frc.robot.subsystems.rollers.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {

  @AutoLog
  public static class SpindexerIOInputs {
    public double velocityRotsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  public default void updateInputs(SpindexerIOInputs inputs) {}

  public default void run(double voltage) {}
}
