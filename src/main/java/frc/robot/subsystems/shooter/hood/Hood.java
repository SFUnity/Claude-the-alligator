package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GeneralUtil;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private double goalPosition;

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    GeneralUtil.logSubsystem(this, "Hood");

    Logger.recordOutput("Hood/Goal", goalPosition);
  }

  public Command setAngle(double angle) {
    return runOnce(() -> goalPosition = angle).andThen(() -> io.setPosition(angle)).withName("updateAngle");
  }

  public boolean atGoal() {
    return Math.abs(inputs.positionDeg - goalPosition) < HoodConstants.angleTolerance;
  }
}
