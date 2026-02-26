package frc.robot.subsystems.rollers.spindexer;

import static frc.robot.subsystems.rollers.spindexer.SpindexerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GeneralUtil;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {

  private SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  private double positionDifference = 0;
  private double startingPosition = 0;

  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);

    positionDifference = inputs.positionRots - startingPosition;

    Logger.recordOutput("Rollers/Spindexer/PositionDifference", positionDifference);
    Logger.processInputs("Rollers/Spindexer", inputs);
    GeneralUtil.logSubsystem(this, "Rollers/Spindexer");
  }

  public Command run() {
    return run(() -> io.run(spindexerSpeedVolts.get())).withName("spindexerRun");
  }

  public Command runBack(double rots) {
    return run(() -> io.run(-(slowSpindexerSpeedVolts.get()))).beforeStarting(() -> startingPosition = inputs.positionRots).until(() -> positionDifference > rots).withName("spindexerRunBack" + rots + "rots");
  }

  public Command stop() {
    return run(() -> io.run(0.0)).withName("spindexerStop");
  }
}
