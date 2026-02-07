package frc.robot.subsystems.shooter;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.util.ShooterUtil.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.flywheels.Flywheels;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.PoseManager;
import frc.robot.util.ShooterUtil;
import frc.robot.util.VirtualSubsystem;

public class Shooter extends VirtualSubsystem {
  private final Flywheels flywheels;
  private final Turret turret;
  private final Hood hood;

  private final PoseManager poseManager;

  private boolean isShooting = false;
  private boolean isScoring = false;

  public Shooter(Flywheels flywheels, Turret turret, Hood hood, PoseManager poseManager) {
    this.flywheels = flywheels;
    this.turret = turret;
    this.hood = hood;
    this.poseManager = poseManager;
  }

  public void periodic() {
    ShooterSolution solution = ShooterUtil.calculateOptimalShot(0, 0, 0);
    turret.setGoalDegs(solution.TurnAngleDeg);
    hood.setAngle(solution.angleDeg);
    flywheels.setVelocity(solution.rpm / 60);
  }

  public boolean readyToShoot() {
    return turret.atGoal() && hood.atGoal() && flywheels.atGoal();
  }

  public Command setShooting(boolean shooting) {
    return runOnce(() -> isShooting = shooting)
        .alongWith(runOnce(() -> turret.setIsShooting(shooting)))
        .alongWith(runOnce(() -> flywheels.setReady(shooting)));
  }

  public Command setScoring(boolean scoring) {
    return runOnce(() -> isScoring = scoring);
  }
}
