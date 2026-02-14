package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

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

  public static Command stopShoot() {
    return Commands.run(() -> Logger.recordOutput("RobotCommands/StopShoot", true))
        .finallyDo((interrupted) -> Logger.recordOutput("RobotCommands/StopShoot", false));
  }

  // TODO make the spindexer stop a bit before the shooter and kicker stops so that the last ball
  // can be fully shot out of the robot
  public static Command stopShoot(Shooter shooter, Kicker kicker, Spindexer spindexer) {
    return shooter
        .setShooting(false)
        .andThen(kicker.stop())
        .andThen(spindexer.stop())
        .withName("StopShoot");
  }

  public static Command readyThenShoot(Shooter shooter, Kicker kicker, Spindexer spindexer) {
<<<<<<< HEAD
=======
    // return
    // spindexer.run().alongWith(kicker.run()).andThen(spindexer.stop()).alongWith(kicker.stop()).withName("ReadyThenShoot");
>>>>>>> 5b2d957 (format)
    return shooter
        .setShooting(true)
        .andThen(
            kicker.runBack().alongWith(spindexer.runBack()).withTimeout(0.2),
            spindexer.stop(),
            kicker.run(),
            waitUntil(shooter::readyToShoot),
            spindexer.run());
  }

  public static Command intake(IntakeRollers intake, IntakePivot intakePivot) {
    return intake.intake().alongWith(intakePivot.lower()).withName("intake");
  }

  public static Command eject(IntakeRollers intake, IntakePivot intakePivot, Spindexer spindexer) {
    return intake.eject().alongWith(intakePivot.lower().withName("eject"));
  }

  public static Command stowIntake(IntakeRollers intake, IntakePivot intakePivot) {
    return intake.stop().alongWith(intakePivot.raise()).withName("stowIntake");
  }

  public static Command jork(IntakeRollers intake, IntakePivot intakePivot) {
    return intake.stop().alongWith(intakePivot.jork()).withName("jork");
  }
}
