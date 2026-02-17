package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.rollers.intakerollers.IntakeRollers;
import frc.robot.subsystems.rollers.kicker.Kicker;
import frc.robot.subsystems.rollers.kicker.Kicker.KickerState;
import frc.robot.subsystems.rollers.spindexer.Spindexer;
import frc.robot.subsystems.shooter.Shooter;

public class RobotCommands {
  // TODO make the spindexer stop a bit before the shooter and kicker stops so that the last ball
  // can be fully shot out of the robot
  public static Command stopShoot(Shooter shooter, Kicker kicker, Spindexer spindexer) {
    return shooter
        .setShooting(false)
        .andThen(kicker.setState(KickerState.STOP))
        .andThen(spindexer.stop())
        .withName("StopShoot");
  }

  public static Command readyThenShoot(Shooter shooter, Kicker kicker, Spindexer spindexer) {
    return shooter
      .setShooting(true)
      .andThen(
        kicker.setState(KickerState.BACKWARDS).alongWith(spindexer.runBack()).withTimeout(0.2),
        spindexer.stop(),
        kicker.setState(KickerState.RUN),
        // TODO also need to wait until the kicker is ready to shoot
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
}
