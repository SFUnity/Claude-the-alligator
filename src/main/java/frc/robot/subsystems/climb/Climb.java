package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GeneralUtil;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {

  private ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  public Climb(ClimbIO io) {
    this.io = io;
    Logger.recordOutput("ClimbPosition", downMeters);
  }

  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Climb", inputs);
    GeneralUtil.logSubsystem(this, "Climb");
  }

  public Command climbUp() {
    return runOnce(() -> Logger.recordOutput("ClimbPosition", upMeters))
        .andThen(() -> io.setPosition(upMeters))
        .withName("climbUp");
  }

  public Command climbDown() {
    return runOnce(() -> Logger.recordOutput("ClimbPosition", downMeters))
        .andThen(() -> io.setPosition(downMeters))
        .withName("climbDown");
  }
}
