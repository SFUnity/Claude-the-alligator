package frc.robot.subsystems.rollers.spindexer;

import static frc.robot.Constants.loopPeriodSecs;
import static frc.robot.subsystems.rollers.spindexer.SpindexerConstants.*;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class SpindexerIOTalonFX implements SpindexerIO {
  private final TalonFX talon = new TalonFX(talonID);

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
  public void run(double voltage) {
    talon.setControl(voltageOut.withOutput(voltage));
  }
}
