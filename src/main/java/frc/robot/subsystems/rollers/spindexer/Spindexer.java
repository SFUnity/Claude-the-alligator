package frc.robot.subsystems.rollers.spindexer;

import static frc.robot.subsystems.rollers.spindexer.SpindexerConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.GeneralUtil;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {

  private SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  private double positionDifference = 0;
  private double startingPosition = 0;

  private final Debouncer currentDebouncer = new Debouncer(0.5, DebounceType.kBoth);

  public Spindexer(SpindexerIO io) {
    this.io = io;
    Logger.recordOutput("Rollers/Spindexer/Jammed", false);
  }

  public void periodic() {
    io.updateInputs(inputs);

    positionDifference = startingPosition - inputs.positionRots;

    Logger.recordOutput("Rollers/Spindexer/PositionDifference", positionDifference);
    Logger.processInputs("Rollers/Spindexer", inputs);
    GeneralUtil.logSubsystem(this, "Rollers/Spindexer");
  }

  private void runUnlessJammed(boolean forward) {
    double extra = forward ? 1 : -1;
    if (currentDebouncer.calculate(inputs.statorCurrentAmps > 40)) {
      io.run(0);
      Logger.recordOutput("Rollers/Spindexer/Jammed", true);
      Leds.getInstance().isJammed = true;
    } else {
      io.run(spindexerSpeedVolts.get() * extra);
      Logger.recordOutput("Rollers/Spindexer/Jammed", false);
      Leds.getInstance().isJammed = false;
    }
  }

  public Command run() {
    return run(() -> runUnlessJammed(true)).withName("spindexerRun");
  }

  public Command runBack(double rots) {
    return runEnd(() -> runUnlessJammed(false), () -> io.run(0))
        .beforeStarting(
            () -> {
              startingPosition = inputs.positionRots;
              positionDifference = 0;
            })
        .until(() -> positionDifference > rots)
        .withName("spindexerRunBack" + rots + "rots");
  }

  public Command runBackUntilJam() {
    return runEnd(
            () -> {
              io.run(-(slowSpindexerSpeedVolts.get()));
            },
            () -> io.run(0))
        // Consider switching to stator current
        .until(() -> currentDebouncer.calculate(inputs.supplyCurrentAmps > 40))
        .withName("spindexerRunBack");
  }

  public Command stop() {
    return run(() -> io.run(0.0)).withName("spindexerStop");
  }
}
