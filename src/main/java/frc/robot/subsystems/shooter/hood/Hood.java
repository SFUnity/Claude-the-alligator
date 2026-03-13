package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.GeneralUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private double goalPosition;
  private boolean isZeroing = false;

  private final Debouncer statorCurrentDebouncer = new Debouncer(0.2, DebounceType.kFalling);

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Hood", inputs);
    GeneralUtil.logSubsystem(this, "Shooter/Hood");

    Logger.recordOutput("Shooter/Hood/Goal", goalPosition);
    Logger.recordOutput("Shooter/Hood/IsZeroing", isZeroing);
    // if (!isZeroing) {}
    io.setPosition(goalPosition);
  }

  public void setAngle(double angle) {
    MathUtil.clamp(angle, minPositionDegs, maxPositionDegs);
    goalPosition = angle;
  }

  @AutoLogOutput(key = "Shooter/Hood/AtGoal")
  public boolean atGoal() {
    return Math.abs(inputs.positionDeg - goalPosition) < HoodConstants.angleTolerance;
  }

  public double getAngleDeg() {
    if (Constants.currentMode == Constants.simMode) {
      return goalPosition;
    }
    return inputs.positionDeg;
  }

  public Command runCurrentZeroing() {
    return run(() -> io.runVolts(-1.0))
        .until(() -> statorCurrentDebouncer.calculate(inputs.statorCurrent > 30.0))
        .finallyDo(
            () -> {
              io.resetEncoder(0.0);
              isZeroing = false;
            })
        .beforeStarting(() -> isZeroing = true)
        .withName("HoodCurrentZeroing");
  }
}
