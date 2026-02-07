package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.intakePivot.IntakePivotConstants.*;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GeneralUtil;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  private final IntakePivotVisualizer measuredVisualizer =
      new IntakePivotVisualizer("Measured", Color.kRed);
  private final IntakePivotVisualizer setpointVisualizer =
      new IntakePivotVisualizer("Setpoint", Color.kBlue);
  private double positionSetpoint = raisedAngle.get();

  private boolean startedIntaking = false;
  private boolean middleOfIntaking = false;

  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/startedIntaking", startedIntaking);
    Logger.recordOutput("Intake/middleOfIntaking", middleOfIntaking);

    // Logs
    measuredVisualizer.update(Degrees.of(inputs.pivotCurrentPositionDeg));
    setpointVisualizer.update(Degrees.of(positionSetpoint));
    Logger.recordOutput("Intake/positionSetpoint", positionSetpoint);
    GeneralUtil.logSubsystem(this, "Intake");
  }

  public Command raise() {
    return run(
        () -> {
          positionSetpoint = raisedAngle.get();
          io.setPivotPosition(positionSetpoint);
        });
  }

  public Command lower() {
    return run(
        () -> {
          positionSetpoint = loweredAngle.get();
          io.setPivotPosition(positionSetpoint);
        });
  }

  public Command runCurrentZeroing() {
    return run(() -> io.runVolts(-1.0))
        .until(() -> inputs.pivotStaterCurrent > 30.0)
        .finallyDo(() -> io.resetEncoder(0.0));
  }

  public Command jork() {
    return raise().andThen(Commands.waitSeconds(jorkTime.get())).andThen(lower()).andThen(Commands.waitSeconds(jorkTime.get()));
  }
}
