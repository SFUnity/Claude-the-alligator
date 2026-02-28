package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class HoodIOTalonFX implements HoodIO {
  private final TalonFX pivot = new TalonFX(hoodMotorID);
  private PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(true);
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private VoltageOut voltageOut = new VoltageOut(0.0);

  public HoodIOTalonFX() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Feedback.SensorToMechanismRatio = gearRatio;

    // config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    // config.Slot0.kS = 0.055;
    // config.Slot0.kG = 0.445;
    // config.Slot0.kV = 1.45;
    config.Slot0.kP = kP.get();
    config.Slot0.kD = kD.get();
    config.MotionMagic.MotionMagicAcceleration = 160.0;
    config.MotionMagic.MotionMagicCruiseVelocity = 100.0;

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60.0;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tryUntilOk(5, () -> pivot.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.appliedVolts = pivot.getMotorVoltage().getValueAsDouble();
    inputs.positionDeg =
        Units.rotationsToDegrees(pivot.getPosition().getValueAsDouble()) * gearRatio;
    inputs.talonRotations = pivot.getRotorPosition().getValueAsDouble();
    inputs.statorCurrent = pivot.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrent = pivot.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void setPosition(double positionDeg) {
    pivot.setControl(
        motionMagicVoltage.withPosition(
            Units.degreesToRotations(positionDeg - minPositionDegs) / gearRatio));
    // positionVoltage.withPosition(
    //     Units.degreesToRotations(positionDeg - minPositionDegs) / gearRatio));

  }

  @Override
  public void resetEncoder(double positionDeg) {
    pivot.setPosition(Units.degreesToRotations(positionDeg));
  }
}
