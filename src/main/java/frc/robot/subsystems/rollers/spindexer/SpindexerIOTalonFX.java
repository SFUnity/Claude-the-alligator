package frc.robot.subsystems.rollers.spindexer;

import static frc.robot.Constants.loopPeriodSecs;
import static frc.robot.subsystems.rollers.spindexer.SpindexerConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class SpindexerIOTalonFX implements SpindexerIO {
  private final TalonFX talon = new TalonFX(talonID);

  private final VoltageOut voltageOut =
      new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(loopPeriodSecs);

  // TODO fill this out
  public SpindexerIOTalonFX() {
    var talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 80.0;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 60.0;
    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    tryUntilOk(5, () -> talon.getConfigurator().apply(talonFXConfigs, 0.25));
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.appliedVolts = talon.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrentAmps = talon.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrentAmps = talon.getSupplyCurrent().getValueAsDouble();
    inputs.velocityRotsPerSec = talon.getVelocity().getValueAsDouble();
    inputs.positionRots = talon.getPosition().getValueAsDouble();
  }

  @Override
  public void run(double voltage) {
    talon.setControl(voltageOut.withOutput(voltage));
  }
}
