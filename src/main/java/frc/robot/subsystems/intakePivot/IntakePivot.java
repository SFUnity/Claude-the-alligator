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

  // TODO remove unused

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

  // TODO use these for the below todo
  // private void lower() {
  //   positionSetpoint = loweredAngle.get();
  //   io.setPivotPosition(positionSetpoint);
  // }

  // private void raise() {
  //   positionSetpoint = raisedAngle.get();
  //   io.setPivotPosition(positionSetpoint);
  // }

  // TODO update "positionSetpoint" so we can track it
  public Command raise() {
    return run(() -> io.setPivotPosition(raisedAngle.get()));
  }

  public Command lower() {
    return run(() -> io.setPivotPosition(loweredAngle.get()));
  }

  public Command jork() {
    return Commands.none();
  }

  // TODO add currentZeroing from Alcatraz
}
