package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.intakePivot.IntakePivotConstants.*;

import edu.wpi.first.math.util.Units;
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

  private boolean shouldBeLowered = false;

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
          if (shouldBeLowered == false && intakeDown()) shouldBeLowered = true;
          if (shouldBeLowered && !intakeDown()) {
            io.runVolts(extraLoweringVoltage.get());
          } else {
            io.setPivotPosition(positionSetpoint);
          }
        })
        .beforeStarting(() -> shouldBeLowered = false)
        .withName("IntakePivotLower");
  }

  public Command runCurrentZeroing() {
    return run(() -> io.runVolts(-3))
        .until(() -> inputs.pivotStaterCurrent > 40)
        .finallyDo(() -> io.resetEncoder(Units.degreesToRotations(-4)))
        .withName("IntakePivotCurrentZeroing");
  }

  public Command runJork() {
    return lower()
        .until(this::intakeDown)
        .andThen(
            Commands.repeatingSequence(
                run(() -> {
                      positionSetpoint = raisedJorkAngle.get();
                      io.runVolts(jorkUpVoltage.get());
                    })
                    .until(
                        () ->
                            inputs.pivotCurrentPositionDeg
                                <= raisedJorkAngle.get() + jorkTolerance.get()),
                run(() -> {
                      positionSetpoint = loweredJorkAngle.get();
                      io.runVolts(jorkDownVoltage.get());
                    })
                    .until(
                        () ->
                            inputs.pivotCurrentPositionDeg
                                >= loweredJorkAngle.get() - jorkTolerance.get())))
        .withName("IntakePivotJork");
  }

  @AutoLogOutput(key = "Controls/IntakeDown")
  public boolean intakeDown() {
    // return Math.abs(inputs.pivotCurrentPositionDeg - loweredAngle.get()) <= 5;
    return Math.abs(loweredAngle.get() - inputs.pivotCurrentPositionDeg) < isDownTolerance.get();
  }

  // public Command zeroOutput() {
  //   return run(() -> io.runVolts(0)).withName("Zero");
  // }
}
