package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.flywheels.Flywheels;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;

public class Shooter extends SubsystemBase {
  private final Flywheels flywheels;
  private final Turret turret;
  private final Hood hood;

  public Shooter(Flywheels flywheels, Turret turret, Hood hood) {
    this.flywheels = flywheels;
    this.turret = turret;
    this.hood = hood;
  }
}
