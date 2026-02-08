package frc.robot.subsystems.shooter;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.subsystems.shooter.ShooterUtil.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.ShooterUtil.*;
import frc.robot.subsystems.shooter.flywheels.Flywheels;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.PoseManager;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class Shooter extends VirtualSubsystem {
  private final Flywheels flywheels;
  private final Turret turret;
  private final Hood hood;

  private final ShooterUtil shooterUtil;

  private final PoseManager poseManager;

  private boolean isShooting = false;
  private boolean isScoring = false;

  public Shooter(Flywheels flywheels, Turret turret, Hood hood, PoseManager poseManager) {
    this.flywheels = flywheels;
    this.turret = turret;
    this.hood = hood;
    this.poseManager = poseManager;
    this.shooterUtil = new ShooterUtil(this.poseManager);

    // TODO add default commands
  }

  public void periodic() {
    Pose3d goalPose = new Pose3d();

    LaunchingParameters solution = shooterUtil.getScoringParameters();

    turret.setTargetDegs(0);
    hood.setAngle(0);
    flywheels.setVelocity(0);

    isScoring = poseManager.getPose().getX() < FieldConstants.LinesVertical.allianceZone;
    Logger.recordOutput("Shooter/isScoring", isScoring);
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
