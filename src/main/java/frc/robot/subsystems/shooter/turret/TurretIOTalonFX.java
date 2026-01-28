package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX talon;
  private final CANcoder encoder1;
  private final CANcoder encoder2;

  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  private final NeutralOut neutralOut = new NeutralOut();

  private final DynamicMotionMagicExpoVoltage motionMagicExpoVoltageShoot =
      new DynamicMotionMagicExpoVoltage(0, shootKV, shootKA).withEnableFOC(true);
  private final DynamicMotionMagicExpoVoltage motionMagicExpoVoltageNoShoot =
      new DynamicMotionMagicExpoVoltage(0, noshootKV, noshootKA).withEnableFOC(true);

  private double angle;

  public TurretIOTalonFX() {
    talon = new TalonFX(motorID);
    encoder1 = new CANcoder(encoder1ID);
    encoder2 = new CANcoder(encoder2ID);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.Inverted =
        motorInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    configs.Feedback.SensorToMechanismRatio = 1;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    talon.getConfigurator().apply(configs);
    angle = getPositionDegs();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.appliedVolts = talon.getMotorVoltage().getValueAsDouble();
    angle = getPositionDegs();
    inputs.positionDegs = angle;
    inputs.velocityDegsPerSec = talon.getVelocity().getValueAsDouble() / gearRatio;
    inputs.currentAmps = talon.getSupplyCurrent().getValueAsDouble();
    inputs.encoder1Degs = encoder1.getPosition().getValueAsDouble();
    inputs.encoder2Degs = encoder2.getPosition().getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }

  @Override
  public void turnTurret(double targetDegs, boolean isShooting) {
    if (isShooting) {
      targetDegs = // check to make sure math works
          (targetDegs < minAngleDegs || targetDegs > maxAngleDegs)
              ? ((360 - Math.abs(targetDegs - centerDegs))
                      * ((targetDegs - centerDegs) / -Math.abs(targetDegs - centerDegs)))
                  + centerDegs
              : targetDegs;
      talon.setControl(
          motionMagicExpoVoltageShoot.withPosition(
              Units.degreesToRotations(targetDegs) * gearRatio));
    } else {
      targetDegs =
          (targetDegs < minBufferAngleDegs || targetDegs > maxBufferAngleDegs)
              ? ((360 - Math.abs(targetDegs - centerDegs))
                      * ((targetDegs - centerDegs) / -Math.abs(targetDegs - centerDegs)))
                  + centerDegs
              : targetDegs;
      talon.setControl(
          motionMagicExpoVoltageNoShoot
              .withPosition( // if positiion is greater than one, what happens
                  Units.degreesToRotations(targetDegs) * gearRatio));
    }
  }

  @Override
  public double getPositionDegs() {
    double truePosition = 0;
    double position1 = encoder1.getPosition().getValueAsDouble();
    double position2 = encoder2.getPosition().getValueAsDouble();

    position1 = Units.rotationsToDegrees(position1);
    position2 = Units.rotationsToDegrees(position2);

    // chinese remainder theorem from claude
    truePosition =
        ((encoder1gear ^ 2 * encoder2gear * 360) * position1
                + (encoder2gear ^ 2 * encoder1gear * 360) * position2)
            % (360 ^ 2 * encoder1gear * encoder2gear);

    return truePosition;
  }
}
