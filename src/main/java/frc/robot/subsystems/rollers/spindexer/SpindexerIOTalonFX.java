package frc.robot.subsystems.rollers.spindexer;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.subsystems.rollers.spindexer.SpindexerConstants.*;
import static frc.robot.Constants.loopPeriodSecs;

public class SpindexerIOTalonFX implements SpindexerIO {
    private final TalonFX talon = new TalonFX(0);

    private final VoltageOut voltageOut =
      new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(loopPeriodSecs);

    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        inputs.appliedVolts = talon.getMotorVoltage().getValueAsDouble();
        inputs.statorCurrentAmps = talon.getStatorCurrent().getValueAsDouble();
        inputs.supplyCurrentAmps = talon.getSupplyCurrent().getValueAsDouble();
        inputs.velocityRotsPerSec = talon.getVelocity().getValueAsDouble();
    }
    @Override
    public void runVolts(double voltage) {
        talon.setControl(voltageOut.withOutput(voltage));
    }
    @Override
    public void stop() {
        talon.setControl(voltageOut.withOutput(0.0));   
    }
}