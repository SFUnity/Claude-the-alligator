package frc.robot.subsystems.shooter.flywheels;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class FlywheelsIOTalonFX implements FlywheelsIO{
    private final TalonFX topMotor = new TalonFX(0);
    private final TalonFX bottomMotor = new TalonFX(0);
    private final VoltageOut topVoltageOut = new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(0);
    private final VoltageOut bottomVoltageOut = new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(0);
    public FlywheelsIOTalonFX(){
        var topTalonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = topTalonFXConfigs.Slot0;
        slot0Configs.kS = 0; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = topTalonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        topMotor.getConfigurator().apply(topTalonFXConfigs);

        var bottomTalonFXConfigs = new TalonFXConfiguration();

        var slot1Configs = bottomTalonFXConfigs.Slot1;
        slot1Configs.kS = 0; // Add 0.25 V output to overcome static friction
        slot1Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot1Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot1Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot1Configs.kI = 0; // no output for integrated error
        slot1Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var bottomMotionMagicConfigs = bottomTalonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        topMotor.getConfigurator().apply(bottomTalonFXConfigs);
    }
    @Override
    public void updateInputs(FlywheelsIOOutputs inputs){
        inputs.topAppliedVolts = topMotor.getMotorVoltage().getValueAsDouble();
        inputs.topCurrentAmps = topMotor.getSupplyCurrent().getValueAsDouble();
        inputs.topVelocityRotsPerSec = topMotor.getVelocity().getValueAsDouble();

        inputs.bottomAppliedVolts = bottomMotor.getMotorVoltage().getValueAsDouble();
        inputs.bottomCurrentAmps = bottomMotor.getSupplyCurrent().getValueAsDouble();
        inputs.bottomVelocityRotsPerSec = bottomMotor.getVelocity().getValueAsDouble();
    }



}
