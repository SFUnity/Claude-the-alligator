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
  private boolean hasGP = false;
  private boolean startedIntaking = false;
  private boolean middleOfIntaking = false;
  public static boolean simHasGP = false;
  private boolean runningIceCream = false;

  private boolean lastGroundAlgae = groundAlgae.get();

  private final LoggedTunableNumber spikeCurrent =
      new LoggedTunableNumber("Intake/spikeCurrent", groundAlgae.get() ? 17 : 17);

  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    filteredCurrent = currentFilter.calculate(inputs.rollersCurrentAmps);
    Logger.recordOutput("Intake/filteredCurrent", filteredCurrent);
    Logger.recordOutput("Intake/startedIntaking", startedIntaking);
    lowered = inputs.pivotCurrentPositionDeg >= loweredAngle.get() / 2;

  

    Logger.recordOutput("Intake/runningIceCream", runningIceCream);

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

  private void rollersIn() {
    io.runRollers(rollersSpeedIn.get());
  }

  private void rollersOut() {
    io.runRollers(-rollersSpeedOut.get());
  }

  public Command runCurrentZeroing() {
    return this.run(() -> io.runPivot(-1.0))
        .until(() -> inputs.pivotCurrentAmps > 30.0)
        .finallyDo(() -> io.resetEncoder(0.0));
  }

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
