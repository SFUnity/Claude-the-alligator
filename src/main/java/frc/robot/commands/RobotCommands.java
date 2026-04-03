package frc.robot.commands;

import static frc.robot.FieldConstants.LinesHorizontal;
import static frc.robot.FieldConstants.LinesVertical;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.rollers.intakerollers.IntakeRollers;
import frc.robot.subsystems.rollers.kicker.Kicker;
import frc.robot.subsystems.rollers.kicker.Kicker.KickerState;
import frc.robot.subsystems.rollers.spindexer.Spindexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class RobotCommands {
  private static final double ejectBackupRots = 0.5;
  // private static final double shootingBackupRots = 0.1;

  // can be fully shot out of the robot
  public static Command stopShoot(Shooter shooter, Kicker kicker, Spindexer spindexer) {
    return Commands.parallel(
            spindexer.stop(),
            Commands.waitSeconds(0.3)
                .andThen(shooter.setShooting(false), kicker.setState(KickerState.STOP)))
        .withName("StopShoot");
  }

  public static Command readyThenShoot(
      Shooter shooter, Kicker kicker, Spindexer spindexer, Supplier<Pose2d> pose2d) {
    BooleanSupplier behindHub =
        () -> {
          Pose2d pose = AllianceFlipUtil.apply(pose2d.get());
          return ((pose.getX() > LinesVertical.neutralZoneNear)
              && (pose.getY() > LinesHorizontal.rightBumpStart)
              && (pose.getY() < LinesHorizontal.leftBumpEnd));
        };
    return Commands.sequence(
            kicker.setState(KickerState.RUN),
            shooter.setShooting(true),
            Commands.waitUntil(shooter::readyToShoot).withTimeout(0.7),
            spindexer
                .runBasic()
                .onlyWhile(() -> !behindHub.getAsBoolean())
                .andThen(spindexer.stop().onlyWhile(behindHub))
                .repeatedly())
        // Commands.waitSeconds(0.1),
        // unjam(spindexer, kicker, shooter))
        .withName("ReadyThenShoot");
  }

  public static Command readyThenShoot(Shooter shooter, Kicker kicker, Spindexer spindexer) {
    return readyThenShoot(shooter, kicker, spindexer, () -> new Pose2d());
  }

  public static Command intake(IntakeRollers intake, IntakePivot intakePivot) {
    return intake.intake().alongWith(intakePivot.lower()).withName("intake");
  }

  public static Command unjam(Spindexer spindexer, Kicker kicker, Shooter shooter) {
    return spindexer
        .runBackUntilJam()
        .deadlineFor(kicker.setState(KickerState.RUN), shooter.testFlywheelsRPM())
        .withName("unjam");
  }

  public static Command stowIntake(IntakeRollers intake, IntakePivot intakePivot) {
    return intake.stop().alongWith(intakePivot.raise()).withName("stowIntake");
  }

  public static Command jork(IntakeRollers intake, IntakePivot intakePivot) {
    return intake.stop().alongWith(intakePivot.runJork()).withName("jork");
    // return intake.stop().alongWith(intakePivot.runStatorJork()).withName("statorjork");
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
                spindexer.runUntilJammed().withTimeout(1),
                kicker.setState(KickerState.RUN).withTimeout(1),
                intake.intake().withTimeout(1),
                shooter.testFlywheelsRPM()))
        .withName("test");
  }
}
