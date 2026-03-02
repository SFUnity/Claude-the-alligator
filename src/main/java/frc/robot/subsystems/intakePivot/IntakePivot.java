package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.intakePivot.IntakePivotConstants.*;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GeneralUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  private final IntakePivotVisualizer measuredVisualizer =
      new IntakePivotVisualizer("Measured", Color.kRed);
  private final IntakePivotVisualizer setpointVisualizer =
      new IntakePivotVisualizer("Setpoint", Color.kBlue);
  private double positionSetpoint = raisedAngle.get();

  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);

    // Logs
    measuredVisualizer.update(Degrees.of(inputs.pivotCurrentPositionDeg));
    setpointVisualizer.update(Degrees.of(positionSetpoint));
    Logger.recordOutput("IntakePivot/positionSetpointDeg", positionSetpoint);
    GeneralUtil.logSubsystem(this, "IntakePivot");
  }

  public Command raise() {
    return run(() -> {
          // if (DriverStation.isAutonomous()) {
          //   io.runVolts(0);
          // } else {
          //   io.runVolts(1.25);
          // }
          positionSetpoint = raisedAngle.get();
          io.setPivotPosition(positionSetpoint);
        })
        .withName("IntakePivotRaise");
  }

  public Command lower() {
    return run(() -> {
          // if (DriverStation.isAutonomous()) {
          //   io.runVolts(0);
          // } else {
          //   io.runVolts(-0.75);
          // }
          positionSetpoint = loweredAngle.get();
          io.setPivotPosition(positionSetpoint);
        })
        .withName("IntakePivotLower");
  }

  public Command runCurrentZeroing() {
    return run(() -> io.runVolts(1.0))
        .until(() -> inputs.pivotSupplyCurrent > 3.0)
        .finallyDo(() -> io.resetEncoder(0.0))
        .withName("IntakePivotCurrentZeroing");
  }

  public Command runJork() {
    return Commands.repeatingSequence(
            run(() -> {
                  positionSetpoint = loweredJorkAngle.get();
                  io.setPivotPosition(positionSetpoint);
                })
                .until(() -> inputs.pivotCurrentPositionDeg >= loweredJorkAngle.get() - 0.1),
            run(() -> {
                  positionSetpoint = raisedJorkAngle.get();
                  io.setPivotPosition(positionSetpoint);
                })
                .until(() -> inputs.pivotCurrentPositionDeg <= raisedJorkAngle.get() + 0.1))
        .withName("IntakePivotJork");
  }

  @AutoLogOutput(key = "Controls/IntakeDown")
  public boolean intakeDown() {
    // return Math.abs(inputs.pivotCurrentPositionDeg - loweredAngle.get()) <= 5;
    return Math.abs(inputs.pivotCurrentPositionDeg) < .5;
  }

  public Command zeroOutput() {
    return run(() -> io.runVolts(0)).withName("Zero");
  }
}
