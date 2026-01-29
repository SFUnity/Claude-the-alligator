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
    // TODO add log subsystem from GeneralUtil + logging framework
  }

  // TODO rename to simply "run" because there are no inputs to the command (it will end up reading
  // spindexer.run())
  public Command run() {
    return run(() -> io.run(spindexerSpeedVolts.get()));
  }

  // TODO rename to "stop" for consistency / readability (it will end up reading spindexer.stop())
  public Command stopSpindexer() {
    return run(() -> io.stop());
  }
}
