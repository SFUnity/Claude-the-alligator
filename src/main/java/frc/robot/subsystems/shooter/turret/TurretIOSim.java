package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

public class TurretIOSim implements TurretIO {
  double rotations = 0;
  double encoder1Rotations = 0;
  double encoder2Rotations = 0;

  public TurretIOSim() {}

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.encoder1Rotations = encoder1Rotations;
    inputs.encoder2Rotations = encoder2Rotations;
    inputs.talonRotations = rotations;
  }

  @Override
  public void turnTurret(double targetRotations, boolean isShooting) {
    rotations = targetRotations;
    encoder1Rotations = targetRotations / gearRatio * (turretGear / encoder1Gear);
    encoder2Rotations = targetRotations / gearRatio * (turretGear / encoder2Gear);
  }
}
