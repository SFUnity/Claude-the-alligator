package frc.robot.subsystems.rollers.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {

    @AutoLog
    public static class SpindexerIOInputs {
        public double velocityRotsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(SpindexerIOInputs inputs) {}

    public default void spin(double velocity) {}

    public default void stop() {}
}
