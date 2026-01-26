package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private double goalDegs = 0;
  private boolean isShooting = false;

  public Turret(TurretIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.turnTurret(goalDegs, isShooting);
  }

  public double getPositionDegs() {
    return io.getPositionDegs();
  }

  public void setGoalDegs(double goalDegs) {
    this.goalDegs = goalDegs;
  }

  public void setIsShooting(boolean isShooting) {
    this.isShooting = isShooting;
  }
}
