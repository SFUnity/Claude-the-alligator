package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;
import com.ctre.phoenix6.signals.GainSchedKpBehaviorValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.MathUtil;
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

    config.ClosedLoopGeneral.GainSchedErrorThreshold = Units.degreesToRotations(.5);
    config.ClosedLoopGeneral.GainSchedKpBehavior = GainSchedKpBehaviorValue.Discontinuous;

    config.Slot0.kS = 0.2;
    config.Slot0.kP = 100;
    config.Slot0.kD = 0.5;
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    config.Slot0.GainSchedBehavior = GainSchedBehaviorValue.ZeroOutput;

    config.MotionMagic.MotionMagicAcceleration = 4.0; // 8
    config.MotionMagic.MotionMagicCruiseVelocity = 2.0; // 4

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60.0;

    tryUntilOk(5, () -> pivot.getConfigurator().apply(config, 0.25));

    this.resetEncoder(0.0);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.appliedVolts = pivot.getMotorVoltage().getValueAsDouble();
    inputs.positionDeg =
        Units.rotationsToDegrees(pivot.getPosition().getValueAsDouble()) + minPositionDegs;
    inputs.talonRotations = pivot.getRotorPosition().getValueAsDouble();
    inputs.statorCurrent = pivot.getStatorCurrent().getValueAsDouble();
    inputs.supplyCurrent = pivot.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void setPosition(double positionDeg) {
    pivot.setControl(
        positionVoltage.withPosition(
            Units.degreesToRotations(
                MathUtil.clamp(
                        positionDeg,
                        minPositionDegs + positionBufferDegs,
                        maxPositionDegs - positionBufferDegs)
                    - minPositionDegs))); // Make it so zero = lowest position
    // positionVoltage.withPosition(
    //     Units.degreesToRotations(positionDeg - minPositionDegs) / gearRatio));

  }

  @Override
  public void runVolts(double volts) {
    pivot.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void resetEncoder(double positionRot) {
    pivot.setPosition(positionRot);
  }
}
