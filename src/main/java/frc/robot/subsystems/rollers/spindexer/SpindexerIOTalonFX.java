package frc.robot.subsystems.rollers.spindexer;

import static frc.robot.Constants.loopPeriodSecs;
// TODO please remove unused import
import static frc.robot.subsystems.rollers.spindexer.SpindexerConstants.*;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class SpindexerIOTalonFX implements SpindexerIO {
  // TODO use the CAN ID from SpindexerConstants
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
  public void run(double voltage) {
    talon.setControl(voltageOut.withOutput(voltage));
  }

  // TODO you can delete this because it doesn't get used anywhere
  @Override
  public void stop() {
    talon.setControl(voltageOut.withOutput(0.0));
  }
}
