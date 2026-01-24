package frc.robot.subsystems.shooter.flywheels;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.google.errorprone.annotations.OverridingMethodsMustInvokeSuper;

public class FlywheelsIOTalonFX implements FlywheelsIO{
    private final TalonFX talonFX;
    private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(0);
    private final MotionMagicVelocityVoltage motionMagicVelocity = new MotionMagicVelocityVoltage(0).withEnableFOC(true);
    public FlywheelsIOTalonFX(double kV, double kP, double kD, int id){
        talonFX = new TalonFX(id);
        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0; 
        slot0Configs.kV = kV; 
        slot0Configs.kA = 0.0; 
        slot0Configs.kP = kP; 
        slot0Configs.kI = 0; 
        slot0Configs.kD = kD; 

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        talonFX.getConfigurator().apply(talonFXConfigs);

        
    }
    @Override
    public void updateInputs(FlywheelsIOOutputs inputs){
        inputs.appliedVolts = talonFX.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = talonFX.getSupplyCurrent().getValueAsDouble();
        inputs.velocityRotsPerSec = talonFX.getVelocity().getValueAsDouble();
    }

    @Override 
    public void runVelocity(double velocity){
        talonFX.setControl(motionMagicVelocity.withVelocity(velocity));
    }

    public void idle(){
       talonFX.setControl(voltageOut.withOutput(0.0)); 
    }




}
