package frc.robot.subsystems.rollers.intakerollers;

import static frc.robot.Constants.loopPeriodSecs;
import static frc.robot.subsystems.rollers.intakerollers.IntakeRollersConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeRollersIOTalonFX implements IntakeRollersIO {
  private final TalonFX rollerMotor = new TalonFX(intakeRollersMotorID);

  private final VoltageOut voltageOut =
      new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(loopPeriodSecs);

  public IntakeRollersIOTalonFX() {
    var talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 80.0;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 60.0;

    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(talonFXConfigs, 0.25));
  }

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {
    inputs.appliedVolts = rollerMotor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = rollerMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    rollerMotor.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void stop() {
    rollerMotor.setControl(voltageOut.withOutput(0));
  }
}
