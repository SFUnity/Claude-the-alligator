package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.drumRadiusMeters;
import static frc.robot.subsystems.climb.ClimbConstants.gearRatio;
import static frc.robot.subsystems.climb.ClimbConstants.kD;
import static frc.robot.subsystems.climb.ClimbConstants.kI;
import static frc.robot.subsystems.climb.ClimbConstants.kP;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ClimbIOTalonFX implements ClimbIO {
  private final TalonFX talon = new TalonFX(0);

  public ClimbIOTalonFX() {
    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI;
    slot0Configs.kD = kD;
    // TODO add neutral mode
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 80.0;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 60.0;
    tryUntilOk(5, () -> talon.getConfigurator().apply(talonFXConfigs, 0.25));
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
    inputs.positionMeters = talon.getPosition().getValueAsDouble() * gearRatio * drumRadiusMeters;
  }

  @Override
  public void setPosition(double rotations) {
    talon.setControl(positionVoltage.withPosition(rotations));
  }
}
