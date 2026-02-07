package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  public static class TurretIOInputs {
    public double encoder1Rotations = 0;
    public double encoder2Rotations = 0;
    public double velocityDegsPerSec = 0;
    public double talonRotations = 0;
    public double appliedVolts = 0;
    // TODO please add stator current as well as supply
    public double currentAmps = 0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void runVolts(double volts) {}

  public default void stop() {}

  public default void turnTurret(double targetDegs, boolean isShooting) {}
}
