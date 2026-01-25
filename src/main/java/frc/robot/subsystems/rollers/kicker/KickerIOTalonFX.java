package frc.robot.subsystems.rollers.kicker;
import static frc.robot.subsystems.rollers.kicker.KickerConstants.*;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;

import static frc.robot.Constants.loopPeriodSecs;

public class KickerIOTalonFX implements KickerIO{
    private final TalonFX rollerMotor = new TalonFX(kickerMotorID);

    private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(loopPeriodSecs);
    
    @Override
    public void updateInputs(KickerIOInputs inputs){
        inputs.appliedVolts = rollerMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = rollerMotor.getSupplyCurrent().getValueAsDouble();
    }
    @Override
    public void runVolts(double volts){
        rollerMotor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void stop(){
        rollerMotor.setControl(voltageOut.withOutput(0));
    }
}
