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
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.appliedVolts = talon.getMotorVoltage().getValueAsDouble();
    inputs.positionDegs = angle;
    inputs.velocityDegsPerSec = talon.getVelocity().getValueAsDouble() / gearRatio;
    inputs.currentAmps = talon.getSupplyCurrent().getValueAsDouble();
    inputs.talonRotations = talon.getPosition().getValueAsDouble();
    inputs.encoder1Rotations = encoder1.getPosition().getValueAsDouble();
    inputs.encoder2Rotations = encoder2.getPosition().getValueAsDouble();
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
    angle = targetDegs;
    if (isShooting) {
      talon.setControl(
          motionMagicExpoVoltageShoot.withPosition(
              Units.degreesToRotations(targetDegs) * gearRatio));
    } else {
      talon.setControl(
          motionMagicExpoVoltageNoShoot
              .withPosition( // if positiion is greater than one, what happens
                  Units.degreesToRotations(targetDegs) * gearRatio));
    }
  }
}
