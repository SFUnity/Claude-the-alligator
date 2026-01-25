package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

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
    }
}
