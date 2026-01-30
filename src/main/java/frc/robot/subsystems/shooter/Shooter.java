package frc.robot.subsystems.shooter;

import frc.robot.util.ShooterUtil.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.VirtualSubsystem;
import frc.robot.subsystems.shooter.flywheels.Flywheels;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;

// TODO I suggest you don't make this a SubsystemBase because then you can have commands that use
// only some of the subsystems. If you still want a periodic method you can use VirtualSubsystem
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
