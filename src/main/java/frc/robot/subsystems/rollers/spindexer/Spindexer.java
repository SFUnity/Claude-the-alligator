package frc.robot.subsystems.rollers.spindexer;

import static frc.robot.subsystems.rollers.spindexer.SpindexerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {

  private SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
  }

  public Command runVolts() {
    return run(() -> io.runVolts(spindexerSpeedVolts.get()));
  }

  public Command stopSpindexer() {
    return run(() -> io.runVolts(0));
  }
}
