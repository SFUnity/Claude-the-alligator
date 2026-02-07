package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
// TODO remove unused imports
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOInputs;

public class HoodIOTalonFX implements HoodIO {
  private final TalonFX pivot = new TalonFX(0);
  private PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);

  public HoodIOTalonFX() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Feedback.SensorToMechanismRatio = 73.5;

    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.Slot0.kS = 0.055;
    config.Slot0.kG = 0.445;
    config.Slot0.kV = 1.45;
    config.Slot0.kP = 35;
    config.Slot0.kD = 0.25;

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60.0;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivot.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.appliedVolts = pivot.getMotorVoltage().getValueAsDouble();
    inputs.positionDeg = Units.rotationsToDegrees(pivot.getPosition().getValueAsDouble());
    inputs.statorCurrent = pivot.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrent = pivot.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void setPosition(double positionDeg) {
    pivot.setControl(positionVoltage.withPosition(Units.degreesToRotations(positionDeg)));
  }
}
