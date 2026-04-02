package frc.robot.subsystems.rollers.kicker;

import static frc.robot.subsystems.rollers.kicker.KickerConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GeneralUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  private Debouncer torqueCurrentDebouncer =
      new Debouncer(torqueCurrentControlDebounce.get(), DebounceType.kFalling);
  private Debouncer atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);
  // private boolean lastTorqueCurrentControl = false;

  boolean torqueCurrentControl = false;
  boolean atGoal = false;

  @AutoLogOutput(key = "Rollers/Kicker/LaunchCount")
  private long launchCount = 0;

  private final double fuelCountDelay = 0.05;
  private final double fuelDistance = 3000000;
  private final Debouncer fuelCountDebouncer = new Debouncer(fuelCountDelay, DebounceType.kRising);
  private boolean fuelCounted = false;

  public enum KickerState {
    RUN,
    STOP,
    BACKWARDS
  }

  private KickerState state = KickerState.STOP;

  public Kicker(KickerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Rollers/Kicker", inputs);
    GeneralUtil.logSubsystem(this, "Rollers/Kicker");

    Logger.recordOutput("Rollers/Kicker/atGoal", atGoal);
    Logger.recordOutput("Rollers/Kicker/torque", torqueCurrentControl);

    if (torqueCurrentControlDebounce.hasChanged(hashCode())) {
      torqueCurrentDebouncer =
          new Debouncer(torqueCurrentControlDebounce.get(), DebounceType.kFalling);
    }
    if (atGoalDebounce.hasChanged(hashCode())) {
      atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);
    }

    switch (state) {
      case STOP:
        io.runVolts(0);
        break;
      case BACKWARDS:
        io.runVolts(-6);
        break;
      case RUN:
        runVelocity();
    }

    if (fuelCountDebouncer.calculate(inputs.laserMeasurementInches < fuelDistance)) {
      if (!fuelCounted) {
        launchCount++;
        fuelCounted = true;
      }
    } else {
      fuelCounted = false;
    }

    Logger.recordOutput("Rollers/Kicker/State", state);
  }

  /** Run closed loop at the specified velocity. */
  private void runVelocity() {
    double velocityRPM = tunableRPMSetpoint.get();

    boolean inToleranceForTorqueControl =
        velocityRPM - inputs.velocityRotsPerMin <= torqueCurrentControlTolerance.get();
    torqueCurrentControl = torqueCurrentDebouncer.calculate(inToleranceForTorqueControl);
    atGoal = atGoalDebouncer.calculate(inToleranceForTorqueControl);

    // if (!torqueCurrentControl && lastTorqueCurrentControl) {
    //   launchCount++;
    // }
    // lastTorqueCurrentControl = torqueCurrentControl;

    // if (inputs.velocityRotsPerMin < velocityRPM) {
    //   if (torqueCurrentControl) {
    //     io.runTorqueControl();
    //   } else {
    //     io.runDutyCycle();
    //   }
    // } else {
    //   io.runSlowDutyCycle(velocityRPM);
    // }

    // sean's pid + torque control solution
    if (!torqueCurrentControl) {
      io.runDutyCycle();
    } else if (velocityRPM - inputs.velocityRotsPerMin >= 10) {
      io.runTorqueControl();
    } else {
      io.runTorqueControl(velocityRPM);
    }

    // io.runTorqueControl(velocityRPM);
  }

  public Command setState(KickerState state) {
    return runOnce(() -> this.state = state).withName("Set " + state);
  }

  public boolean atGoal() {
    return atGoal;
  }
}
