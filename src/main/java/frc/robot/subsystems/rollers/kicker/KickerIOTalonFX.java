package frc.robot.subsystems.rollers.kicker;

import static frc.robot.Constants.loopPeriodSecs;
import static frc.robot.subsystems.rollers.kicker.KickerConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class KickerIOTalonFX implements KickerIO {
  private final TalonFX rollerMotor = new TalonFX(kickerMotorID);
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withEnableFOC(true);

  private final VoltageOut voltageOut =
      new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(loopPeriodSecs);

  public KickerIOTalonFX() {
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0;
    slot0Configs.kV = kV.get();
    slot0Configs.kA = kA.get();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = 0;
    slot0Configs.kD = kD.get();
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 80.0;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 60.0;

    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(talonFXConfigs, 0.25));
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
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

  @Override
  public void runVelocity(double rps) {
    rollerMotor.setControl(velocityVoltage.withVelocity((rps / 60)));
  }
}
