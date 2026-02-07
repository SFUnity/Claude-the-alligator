package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.rollers.intakerollers.IntakeRollers;
import frc.robot.subsystems.rollers.kicker.Kicker;
import frc.robot.subsystems.rollers.spindexer.Spindexer;
import frc.robot.subsystems.shooter.Shooter;
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
    return Commands.run(() -> Logger.recordOutput("RobotCommands/Shoot", true))
        .finallyDo((interrupted) -> Logger.recordOutput("RobotCommands/Shoot", false));
  }

  public static Command shoot(Shooter shooter, Kicker kicker, Spindexer spindexer) {
    return shooter
        .setShooting(true)
        .andThen(kicker.runVolts())
        .andThen(spindexer.run().onlyIf(() -> shooter.readyToShoot()));
  }

  public static Command stopShoot() {
    return Commands.run(() -> Logger.recordOutput("RobotCommands/StopShoot", true))
        .finallyDo((interrupted) -> Logger.recordOutput("RobotCommands/StopShoot", false));
  }

  public static Command stopShoot(Shooter shooter, Kicker kicker, Spindexer spindexer) {
    return shooter.setShooting(false).andThen(kicker.stop()).andThen(spindexer.stop());
  }

  public static Command intake(IntakeRollers intake, IntakePivot intakePivot) {
    return intake.intake().alongWith(intakePivot.lower());
  }

  public static Command eject(IntakeRollers intake, IntakePivot intakePivot) {
    return intake.eject().alongWith(intakePivot.lower());
  }

  public static Command stowIntake(IntakeRollers intake, IntakePivot intakePivot) {
    return intake.stop().alongWith(intakePivot.raise());
  }

  public static Command jork(IntakeRollers intake, IntakePivot intakePivot) {
    return intake.stop().alongWith(intakePivot.jork());
  }
}
