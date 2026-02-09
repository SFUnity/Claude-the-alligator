package frc.robot.subsystems.rollers.spindexer;

import static frc.robot.subsystems.rollers.spindexer.SpindexerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GeneralUtil;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {

  private SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Spindexer", inputs);
    GeneralUtil.logSubsystem(this, "Spindexer");
  }

  // TODO put make an eject command
  public Command run() {
    return run(() -> io.run(spindexerSpeedVolts.get())).withName("spindexerRun");
  }

  public Command stop() {
    return run(() -> io.run(0.0)).withName("spindexerStop");
  }
}
