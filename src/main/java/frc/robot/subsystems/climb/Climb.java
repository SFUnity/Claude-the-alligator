package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GeneralUtil;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {

  private ClimbIO io;
  private boolean isUp = false;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  public Climb(ClimbIO io) {
    this.io = io;
    Logger.recordOutput("Climb/ClimbSetPoint", downMeters);
  }

  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Climb", inputs);
    GeneralUtil.logSubsystem(this, "Climb");

    if (isUp) {
      Logger.recordOutput("Climb/ClimbSetPoint", upMeters);
      io.setPosition(upMeters);
    } else {
      Logger.recordOutput("Climb/ClimbSetPoint", downMeters);
      io.setPosition(downMeters);
    }
  }

  public Command climbUp() {
    return run(() -> isUp = true).withName("climbUp");
  }

  public Command climbDown() {
    return run(() -> isUp = false).withName("climbDown");
  }

  public Command upVolts() {
    return run(() -> io.climbVolts(8.0));
  }

  public Command downVolts() {
    return run(() -> io.climbVolts(8.0));
  }
}
