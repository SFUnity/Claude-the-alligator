package frc.robot.subsystems.shooter.flywheels;

public interface FlywheelsIO {
    public static class FlywheelsIOOutputs{
        public double topVelocityRotsPerSec = 0.0;
        public double topAppliedVolts = 0.0;
        public double topCurrentAmps = 0.0;

        public double bottomVelocityRotsPerSec = 0.0;
        public double bottomAppliedVolts = 0.0;
        public double bottomCurrentAmps = 0.0;
    }
    public default void updateInputs(FlywheelsIOOutputs inputs){}

    public default void runVolts(){}
}
