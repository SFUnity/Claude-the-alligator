package frc.robot.subsystems.climb;

import static frc.robot.Constants.loopPeriodSecs;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class climbIOTalonFX implements climbIO {
  private final TalonFX talon = new TalonFX(0);

  private final VoltageOut voltageOut =
      new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(loopPeriodSecs);

  @Override
  public void updateInputs(climbIOInputs inputs) {
    inputs.appliedVolts = talon.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrentAmps = talon.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrentAmps = talon.getSupplyCurrent().getValueAsDouble();
    inputs.velocityRotsPerSec = talon.getVelocity().getValueAsDouble();
  }

  @Override
  public void runVolts(double voltage) {
    talon.setControl(voltageOut.withOutput(voltage));
  }
}
