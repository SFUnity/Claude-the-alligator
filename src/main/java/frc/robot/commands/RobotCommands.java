package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.rollers.intakerollers.IntakeRollers;
import frc.robot.subsystems.rollers.kicker.Kicker;
import frc.robot.subsystems.rollers.kicker.Kicker.KickerState;
import frc.robot.subsystems.rollers.spindexer.Spindexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LoggedTunableNumber;

public class RobotCommands {
  public static final LoggedTunableNumber ejectBackupRots =
      new LoggedTunableNumber("RobotCommands/ejectBackupRots", 0.25);
  public static final LoggedTunableNumber shootingBackupRots =
      new LoggedTunableNumber("RobotCommands/shootingBackupRots", 0.1);

  // can be fully shot out of the robot
  public static Command stopShoot(Shooter shooter, Kicker kicker, Spindexer spindexer) {
    return spindexer
        .stop()
        .andThen(waitSeconds(0.3))
        .andThen(kicker.setState(KickerState.STOP))
        .andThen(shooter.setShooting(false))
        .withName("StopShoot");
  }

  public static Command readyThenShoot(Shooter shooter, Kicker kicker, Spindexer spindexer) {
    return shooter
        .setShooting(true)
        .andThen(
            kicker
                .setState(KickerState.BACKWARDS)
                .alongWith(spindexer.runBack(shootingBackupRots.get())),
            spindexer.stop(),
            kicker.setState(KickerState.RUN),
            waitUntil(kicker::atGoal),
            waitUntil(shooter::readyToShoot),
            spindexer.run())
            .withName("ReadyThenShoot");
  }

  public static Command intake(IntakeRollers intake, IntakePivot intakePivot) {
    return intake.intake().alongWith(intakePivot.lower()).withName("intake");
  }

  public static Command eject(IntakeRollers intake, IntakePivot intakePivot, Spindexer spindexer) {
    return intake
        .eject()
        .alongWith(intakePivot.lower())
        .deadlineFor(spindexer.runBack(ejectBackupRots.get()))
        .withName("eject");
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
