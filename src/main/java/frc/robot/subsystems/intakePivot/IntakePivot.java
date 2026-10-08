package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.robot.subsystems.intakePivot.IntakePivotConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
  private double statorJorkAngle = 0;

  private Debouncer statorDebounce = new Debouncer(0.2, DebounceType.kRising);

  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

  private boolean shouldBeLowered = false;
  public double prezeroedAngle = 100;

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);

    Logger.recordOutput("IntakePivot/shouldBeLowered", shouldBeLowered);

    // Logs
    measuredVisualizer.update(Degrees.of(inputs.currentPositionDeg));
    setpointVisualizer.update(Degrees.of(positionSetpoint));
    Logger.recordOutput("IntakePivot/positionSetpointDeg", positionSetpoint);
    Logger.recordOutput("IntakePivot/prezeroedAngle", prezeroedAngle);
    GeneralUtil.logSubsystem(this, "IntakePivot");
  }

  public Command idle() {
    return run(() -> io.runVolts(0)).withName("Idle");
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
          boolean useLowerExtraVoltage = loweredAngle.get() - inputs.currentPositionDeg > 3;
          Logger.recordOutput("IntakePivot/lowerExtraVoltage", useLowerExtraVoltage);
          positionSetpoint = loweredAngle.get();
          if (shouldBeLowered == false && intakeDown()) shouldBeLowered = true;
          if (shouldBeLowered && useLowerExtraVoltage) {
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
        .until(() -> inputs.statorCurrent > 40)
        .andThen(waitSeconds(0.2))
        .finallyDo(
            () -> {
              prezeroedAngle = inputs.currentPositionDeg;
              io.resetEncoder(Units.degreesToRotations(-10));
            })
        .withName("IntakePivotCurrentZeroing");
  }

  public Command runJorkUnlessJammed() {
    return Commands.repeatingSequence(
            run(() -> {
                  positionSetpoint = raisedJorkAngle.get();
                  io.runVolts(jorkUpVoltage.get());
                })
                .until(
                    () ->
                        inputs.currentPositionDeg <= raisedJorkAngle.get() + jorkTolerance.get()
                            || inputs.statorCurrent > 40)
                .unless(() -> inputs.statorCurrent > 40),
            run(() -> {
                  positionSetpoint = loweredJorkAngle.get();
                  io.runVolts(jorkDownVoltage.get());
                })
                .until(
                    () ->
                        inputs.currentPositionDeg >= loweredJorkAngle.get() - jorkTolerance.get()))
        .unless(() -> inputs.statorCurrent > 40)
        .withName("IntakePivotJork");
  }

  public Command runJork() {
    return Commands.repeatingSequence(
            run(() -> {
                  positionSetpoint = raisedJorkAngle.get();
                  io.runVolts(jorkUpVoltage.get());
                })
                .until(
                    () -> inputs.currentPositionDeg <= raisedJorkAngle.get() + jorkTolerance.get()),
            run(() -> {
                  positionSetpoint = loweredJorkAngle.get();
                  io.runVolts(jorkDownVoltage.get());
                })
                .until(
                    () ->
                        inputs.currentPositionDeg >= loweredJorkAngle.get() - jorkTolerance.get()))
        // return lower()
        //     .until(this::intakeDown)
        //     .andThen(
        //         run(() -> {
        //               positionSetpoint = raisedJorkAngle.get();
        //               io.runVolts(jorkUpVoltage.get());
        //             })
        //             .until(
        //                 () -> inputs.currentPositionDeg <= raisedJorkAngle.get() +
        // jorkTolerance.get()))
        //     .finallyDo(() -> io.runVolts(0))
        .withName("IntakePivotJork");
  }

  public Command runStatorJork() {
    return Commands.repeatingSequence(
        run(() -> io.runVolts(jorkUpVoltage.get()))
            .until(() -> statorDebounce.calculate(inputs.statorCurrent >= 20.0))
            .finallyDo(() -> statorJorkAngle = inputs.currentPositionDeg),
        run(() -> {
              positionSetpoint = statorJorkAngle + 15;
              io.runVolts(jorkDownVoltage.get());
            })
            .until(() -> inputs.currentPositionDeg >= positionSetpoint - jorkTolerance.get()));
  }

  @AutoLogOutput(key = "IntakePivot/IntakeDown")
  public boolean intakeDown() {
    // return Math.abs(inputs.currentPositionDeg - loweredAngle.get()) <= 5;
    return loweredAngle.get() - inputs.currentPositionDeg < isDownTolerance.get();
  }

  // public Command zeroOutput() {
  //   return run(() -> io.runVolts(0)).withName("Zero");
  // }
}
