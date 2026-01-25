package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
      io.updateInputs(inputs);
  }

  public Command setAngle(double angle) {
    return run(() -> io.setPosition(angle)).withName("updateAngle");
  }

  public boolean isAtAngle(){
    return Math.abs(inputs.positionDeg - inputs.goalPosition) < HoodConstants.angleTolerance;
  }
}
