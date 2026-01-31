package frc.robot.subsystems.rollers.intakerollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.intake.IntakeIOInputsAutoLogged;

import static frc.robot.subsystems.rollers.intakerollers.IntakeRollersConstants.*;

import org.littletonrobotics.junction.Logger;

public class IntakeRollers extends SubsystemBase {
  private final IntakeRollersIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public IntakeRollers(IntakeRollersIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("intake", inputs);
    // TODO add log subsystem from GeneralUtil
  }

  public Command intake() {
    return run(() -> io.runVolts(intakeSpeedVolts.get()));
  }

  public Command stop() {
    return run(() -> io.runVolts(0));
  }

  public Command eject() {
    return run(() -> io.runVolts(-intakeSpeedVolts.get()));
  }
}
