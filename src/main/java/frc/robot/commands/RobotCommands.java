package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.rollers.intakerollers.IntakeRollers;
import frc.robot.subsystems.rollers.kicker.Kicker;
import frc.robot.subsystems.rollers.kicker.Kicker.KickerState;
import frc.robot.subsystems.rollers.spindexer.Spindexer;
import frc.robot.subsystems.shooter.Shooter;

public class RobotCommands {
  private static final double ejectBackupRots = 0.5;
  // private static final double shootingBackupRots = 0.1;

  // can be fully shot out of the robot
  public static Command stopShoot(Shooter shooter, Kicker kicker, Spindexer spindexer) {
    return Commands.parallel(
            spindexer.stop(),
            waitSeconds(0.3)
                .andThen(
                    shooter.setShooting(false),
                    kicker.setState(KickerState.STOP))) // ! this is a run cmd watch out
        .withName("StopShoot");
  }

  public static Command readyThenShoot(Shooter shooter, Kicker kicker, Spindexer spindexer) {
    return Commands.parallel(
            kicker.setState(KickerState.RUN),
            shooter.setShooting(true),
            waitSeconds(1).andThen(spindexer.run()))
        .withName("ReadyThenShoot");
  }

  public static Command intake(IntakeRollers intake, IntakePivot intakePivot) {
    return intake.intake().alongWith(intakePivot.lower()).withName("intake");
  }

  public static Command unjam(Spindexer spindexer, Kicker kicker, Shooter shooter) {
    return spindexer
        .runBack(ejectBackupRots)
        .andThen(kicker.setState(KickerState.RUN))
        .alongWith(shooter.setShooting(true))
        .withName("unjam");
  }

  public static Command stowIntake(IntakeRollers intake, IntakePivot intakePivot) {
    return intake.stop().alongWith(intakePivot.raise()).withName("stowIntake");
  }

  public static Command jork(IntakeRollers intake, IntakePivot intakePivot) {
    return intake.stop().alongWith(intakePivot.runJork()).withName("jork");
  }

  public static Command readyThenShootWithJork(
      Shooter shooter,
      Kicker kicker,
      Spindexer spindexer,
      IntakeRollers intake,
      IntakePivot intakePivot) {
    return readyThenShoot(shooter, kicker, spindexer)
        .deadlineFor(jork(intake, intakePivot))
        .withName("readyThenShootWithJork");
  }

  public static Command test(
      Shooter shooter,
      Kicker kicker,
      Spindexer spindexer,
      IntakeRollers intakeRollers,
      IntakePivot intakePivot,
      IntakeRollers intake) {
    return Commands.sequence(
            shooter.testTurret(),
            shooter.testHood(),
            intakePivot.lower().withTimeout(1),
            intakePivot.raise().withTimeout(1))
        .andThen(
            Commands.parallel(
                spindexer.run().withTimeout(1),
                kicker.setState(KickerState.RUN).withTimeout(1),
                intake.intake().withTimeout(1),
                shooter.testFlywheelsRPM()))
        .withName("test");
  }
}
