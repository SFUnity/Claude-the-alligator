package frc.robot.subsystems.shooter;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.flywheels.Flywheels;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.VirtualSubsystem;

public class Shooter extends VirtualSubsystem {
  private final Flywheels flywheels;
  private final Turret turret;
  private final Hood hood;

  private boolean isShooting = false;
  private boolean isScoring = false;

  public Shooter(Flywheels flywheels, Turret turret, Hood hood) {
    this.flywheels = flywheels;
    this.turret = turret;
    this.hood = hood;
  }

  public void periodic() {}

  public Command setShooting(boolean shooting) {
    return runOnce(() -> isShooting = shooting);
  }

  public Command setScoring(boolean scoring) {
    return runOnce(() -> isScoring = scoring);
  }
}
