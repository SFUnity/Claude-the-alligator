package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.intakePivot.IntakePivotConstants.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GeneralUtil;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  private final IntakePivotVisualizer measuredVisualizer =
      new IntakePivotVisualizer("Measured", Color.kRed);
  private final IntakePivotVisualizer setpointVisualizer =
      new IntakePivotVisualizer("Setpoint", Color.kBlue);
  private double positionSetpoint = raisedAngle.get();

  private final LinearFilter currentFilter = LinearFilter.movingAverage(4);
  private double filteredCurrent;

  private boolean lowered = false;
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
    Logger.recordOutput("Intake/filteredCurrent", filteredCurrent);
    Logger.recordOutput("Intake/startedIntaking", startedIntaking);
    lowered = inputs.pivotCurrentPositionDeg >= loweredAngle.get() / 2;

    // Logs
    measuredVisualizer.update(Degrees.of(inputs.pivotCurrentPositionDeg));
    setpointVisualizer.update(Degrees.of(positionSetpoint));
    Logger.recordOutput("Intake/positionSetpoint", positionSetpoint);
    GeneralUtil.logSubsystem(this, "Intake");
  }

  

  // private void lower() {
  //   positionSetpoint = loweredAngle.get();
  //   io.setPivotPosition(positionSetpoint);
  // }

  private void setL1() {
    positionSetpoint = l1Angle.get();
    io.setPivotPosition(positionSetpoint);
  }

  // private void raise() {
  //   positionSetpoint = raisedAngle.get();
  //   io.setPivotPosition(positionSetpoint);
  // }
 

  public Command raise() {
    return Commands.none();
  }

  public Command lower() {
    return Commands.none();
  }

  public Command jork() {
    return Commands.none();
  }
}
