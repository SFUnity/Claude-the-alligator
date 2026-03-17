package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

  private final NeutralOut neutralOut = new NeutralOut();
  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  private final MotionMagicVoltage motionMagicVoltage =
      new MotionMagicVoltage(0).withEnableFOC(true);

  double talonVelocity;
  double talonRotations;

  public TurretIOTalonFX() {
    talon = new TalonFX(motorID);
    encoder1 = new CANcoder(encoder1ID);
    encoder2 = new CANcoder(encoder2ID);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.Inverted =
        motorInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    // configs.Feedback.SensorToMechanismRatio = gearRatio;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.CurrentLimits.StatorCurrentLimit = 80.0;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 60.0;
    configs.Slot0.kP = 0.21;
    configs.Slot0.kV = 0.07;
    configs.Slot0.kD = 0;
    configs.MotionMagic.MotionMagicCruiseVelocity = 360;
    configs.MotionMagic.MotionMagicAcceleration = 360;

    tryUntilOk(5, () -> talon.getConfigurator().apply(configs, 0.25));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    talonRotations = talon.getPosition().getValueAsDouble();
    talonVelocity = talon.getVelocity().getValueAsDouble();
    inputs.appliedVolts = talon.getMotorVoltage().getValueAsDouble();
    inputs.velocityDegsPerSec = Units.rotationsToDegrees(talonVelocity);
    inputs.currentAmps = talon.getSupplyCurrent().getValueAsDouble();
    inputs.statorCurrent = talon.getStatorCurrent().getValueAsDouble();
    inputs.talonRotations = talonRotations;
    inputs.encoder1Rotations = encoder1.getPosition().getValueAsDouble();
    inputs.encoder2Rotations = encoder2.getPosition().getValueAsDouble();
    inputs.encoder1Disconnected = !encoder1.isConnected();
    inputs.encoder2Disconnected = !encoder2.isConnected();
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }

  @Override
  public void turnTurret(double targetRotations) {
    // talon.setControl(
    //     voltageOut.withOutput(
    //         (Units.rotationsToRadians(targetRotations) -
    // Units.rotationsToRadians(talonRotations))
    //                 * kA
    //             + (Units.degreesToRadians(targetVelocity) -
    // Units.rotationsToRadians(talonVelocity))
    //                 * kV));
    // TODO change if dist is further than a point
    talon.setControl(motionMagicVoltage.withPosition(targetRotations));
  }

  @Override
  public void resetMotorEncoder(double rotations) {
    talon.setPosition(rotations);
  }
}
