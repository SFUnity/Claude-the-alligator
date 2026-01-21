package frc.robot.subsystems.shooter.flywheels;

public interface FlywheelsIO {
    public static class FlywheelsIOOutputs{
        public double velocityRotsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;


    }
    public default void updateInputs(FlywheelsIOOutputs inputs){}

    public default void runVeloctiy(double velocity){}
}
