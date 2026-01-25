package frc.robot.subsystems.rollers.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.rollers.kicker.KickerIO.KickerIOInputs;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs{
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double velocity = 0.0;
    }

    public default void updateInputs(KickerIOInputs inputs) {}

    public default void runVolts(double volts) {}

    public default void stop() {}
}
