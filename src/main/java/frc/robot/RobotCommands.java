package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.rollers.kicker.Kicker;
import frc.robot.subsystems.rollers.kicker.KickerConstants;
import frc.robot.subsystems.rollers.spindexer.Spindexer;
import frc.robot.subsystems.shooter.flywheels.Flywheels;
import frc.robot.subsystems.shooter.flywheels.FlywheelsConstants;
import frc.robot.subsystems.shooter.hood.Hood;
import org.littletonrobotics.junction.Logger;

public class RobotCommands {
  public static Command intake() {
    return Commands.run(() -> Logger.recordOutput("RobotCommands/Intake", true))
        .finallyDo((interrupted) -> Logger.recordOutput("RobotCommands/Intake", false));
  }

  public static Command stopIntake() {
    return Commands.run(() -> Logger.recordOutput("RobotCommands/StopIntake", true))
        .finallyDo((interrupted) -> Logger.recordOutput("RobotCommands/StopIntake", false));
  }

  public static Command climbExtend() {
    return Commands.run(() -> Logger.recordOutput("RobotCommands/ClimbExtend", true))
        .finallyDo((interrupted) -> Logger.recordOutput("RobotCommands/ClimbExtend", false));
  }

  public static Command climbRetract() {
    return Commands.run(() -> Logger.recordOutput("RobotCommands/ClimbRetract", true))
        .finallyDo((interrupted) -> Logger.recordOutput("RobotCommands/ClimbRetract", false));
  }

  public static Command shoot() {
    return Commands.run(() -> Logger.recordOutput("RobotCommands/Shooter", true))
        .finallyDo((interrupted) -> Logger.recordOutput("RobotCommands/Shooter", false));
  }

  public static Command shoot(Flywheels flywheels, Kicker kicker, Hood hood, Spindexer spindexer) {
    return new SequentialCommandGroup(
        flywheels
            .setVelocity(FlywheelsConstants.shootVelocity)
            .withDeadline(kicker.runVolts())
            .withTimeout(KickerConstants.spinupTime),
        spindexer.runVolts().onlyIf(hood::isAtAngle));
  }

  public static Command stopShoot() {
    return Commands.run(() -> Logger.recordOutput("RobotCommands/Shoot", true))
        .finallyDo((interrupted) -> Logger.recordOutput("RobotCommands/Shoot", false));
  }

  public static Command sequence(Command... commands) {
    return new SequentialCommandGroup(commands);
  }
}
