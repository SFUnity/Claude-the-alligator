package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX talon;
  private final CANcoder encoder1;
  private final CANcoder encoder2;

  public TurretIOTalonFX() {
    talon = new TalonFX(motorID);
    encoder1 = new CANcoder(encoder1ID);
    encoder2 = new CANcoder(encoder2ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = 
            motorInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        config.Feedback.SensorToMechanismRatio = 1;
        // config.Slot0
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.appliedVolts = talon.getMotorVoltage().getValueAsDouble();
        inputs.positionDegs = getPositionDegs();
        inputs.velocityDegsPerSec = talon.getVelocity().getValueAsDouble()/gearRatio;
        inputs.currentAmps = talon.getSupplyCurrent().getValueAsDouble();
        inputs.encoder1Degs = encoder1.getPosition().getValueAsDouble();
        inputs.encoder2Degs = encoder2.getPosition().getValueAsDouble();
    }

    @Override
    public double getPositionDegs() {
        double truePosition = 0;
        double position1 = encoder1.getPosition().getValueAsDouble();
        double position2 = encoder2.getPosition().getValueAsDouble();

        position1 = Units.rotationsToDegrees(position1);
        position2 = Units.rotationsToDegrees(position2);

        truePosition = ((encoder1gear^2*encoder2gear*360)*position1+(encoder2gear^2*encoder1gear*360)*position2)%(360^2*encoder1gear*encoder2gear);

        return truePosition;
    }
}
