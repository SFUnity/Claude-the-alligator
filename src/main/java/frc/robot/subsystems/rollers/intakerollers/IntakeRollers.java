package frc.robot.subsystems.rollers.intakerollers;

import static frc.robot.subsystems.rollers.intakerollers.IntakeRollersConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GeneralUtil;
import org.littletonrobotics.junction.Logger;

public class IntakeRollers extends SubsystemBase {
  private final IntakeRollersIO io;
  private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();

  public IntakeRollers(IntakeRollersIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeRollers", inputs);
    GeneralUtil.logSubsystem(this, "IntakeRollers");
  }

  public Command intake() {
    return run(() -> io.runVolts(intakeRollersSpeedVolts.get())).withName("intake");
  }

  public Command stop() {
    return run(() -> io.runVolts(0)).withName("stop");
  }

  public Command eject() {
    return run(() -> io.runVolts(-intakeRollersSpeedVolts.get())).withName("eject");
  }
}
