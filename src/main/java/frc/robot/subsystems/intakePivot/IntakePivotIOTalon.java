package frc.robot.subsystems.intakePivot;

import static frc.robot.subsystems.intakePivot.IntakePivotConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class IntakePivotIOTalon implements IntakePivotIO {
  private final TalonFX pivot = new TalonFX(pivotID);
  private PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);
  private VoltageOut voltageOut = new VoltageOut(0.0);
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0).withEnableFOC(true);

  public IntakePivotIOTalon() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Feedback.SensorToMechanismRatio = gearRatio;

    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.Slot0.kP = 35;
    config.Slot0.kD = 0.25;
    config.MotionMagic.MotionMagicAcceleration = 160.0;
    config.MotionMagic.MotionMagicCruiseVelocity = 100.0;

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60.0;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    tryUntilOk(5, () -> pivot.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.pivotCurrentPositionDeg =
        Units.rotationsToDegrees(pivot.getPosition().getValueAsDouble());
    inputs.pivotAppliedVolts = pivot.getMotorVoltage().getValueAsDouble();
    inputs.pivotStaterCurrent = pivot.getStatorCurrent().getValueAsDouble();
    inputs.pivotSupplyCurrent = pivot.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void setPivotPosition(double setpointDeg) {
    pivot.setControl(motionMagicVoltage.withPosition(setpointDeg));
  }

  @Override
  public void runVolts(double volts) {
    pivot.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void resetEncoder(double position) {
    pivot.setPosition(position);
  }
}
