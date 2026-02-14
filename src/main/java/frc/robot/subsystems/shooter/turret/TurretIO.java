package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  public static class TurretIOInputs {
    public double encoder1Rotations = 0;
    public double encoder2Rotations = 0;
    public boolean encoder1Disconnected = false;
    public boolean encoder2Disconnected = false;
    public double velocityDegsPerSec = 0;
    public double talonRotations = 0;
    public double appliedVolts = 0;
    // TODO please add stator current as well as supply
    public double currentAmps = 0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void stop() {}

  public default void runVolts(double volts) {}

  public default void turnTurret(double targetRotations, boolean isShooting) {}
}
