package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimbIOTalonFX implements ClimbIO {
  private final TalonFX talon = new TalonFX(0);

  public ClimbIOTalonFX() {
    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = kP.get();
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    
    
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonFXConfigs.Feedback.RotorToSensorRatio = gearRatio * (drumRadiusMeters*Math.PI*2);
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 80.0;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 60.0;
   
    tryUntilOk(5, () -> talon.getConfigurator().apply(talonFXConfigs, 0.25));
  }

  final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.appliedVolts = talon.getMotorVoltage().getValueAsDouble();
    inputs.statorCurrentAmps = talon.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrentAmps = talon.getSupplyCurrent().getValueAsDouble();
    inputs.velocityRotsPerSec = talon.getVelocity().getValueAsDouble();
    inputs.positionMeters = talon.getPosition().getValueAsDouble();
  }

  @Override
  public void setPosition(double meters) {
    talon.setControl(positionVoltage.withPosition(meters));
  }
}
