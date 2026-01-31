package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ClimbIOTalonFX implements ClimbIO {
  private final TalonFX talon = new TalonFX(0);

  public ClimbIOTalonFX() {
    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = 0;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 80.0;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 60.0;
    talon.getConfigurator().apply(talonFXConfigs);
  }

  // private final VoltageOut voltageOut =
  //    new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(loopPeriodSecs);
  final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.appliedVolts = talon.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrentAmps = talon.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrentAmps = talon.getSupplyCurrent().getValueAsDouble();
    inputs.velocityRotsPerSec = talon.getVelocity().getValueAsDouble();
  }

  @Override
  public void setPosition(double rotations) {
    talon.setControl(positionVoltage.withPosition(rotations));
  }
}
