package frc.robot.subsystems.intakePivot;

import static frc.robot.subsystems.intakePivot.IntakePivotConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class IntakePivotIOTalon implements IntakePivotIO {
  private final TalonFX pivot = new TalonFX(pivotID);
  private PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);

  public IntakePivotIOTalon() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Feedback.SensorToMechanismRatio = 73.5;

    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.Slot0.kP = 35;
    config.Slot0.kD = 0.25;

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60.0;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivot.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.pivotCurrentPositionDeg = pivot.getPosition().getValueAsDouble();
    inputs.pivotAppliedVolts = pivot.getMotorVoltage().getValueAsDouble();
    inputs.pivotStaterCurrent = pivot.getStatorCurrent().getValueAsDouble();
    inputs.pivotSupplyCurrent = pivot.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void setPivotPosition(double setpointDeg) {
    pivot.setControl(positionVoltage.withPosition(Units.degreesToRotations(setpointDeg)));
  }

  @Override
  public void resetEncoder(double position) {
    pivot.setPosition(position);
  }
}
