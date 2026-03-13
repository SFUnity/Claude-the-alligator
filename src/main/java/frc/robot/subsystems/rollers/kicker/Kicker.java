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
  private boolean lastTorqueCurrentControl = false;

  boolean torqueCurrentControl = false;
  boolean atGoal = false;

  @AutoLogOutput(key = "Rollers/Kicker/LaunchCount")
  private long launchCount = 0;

  private final double fuelCountDelay = 0.2;
  private final double fuelDistance = 0.5;

  private Debouncer fuelCountDebouncer = new Debouncer(fuelCountDelay, DebounceType.kRising);

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
        io.runVolts(-1);
        break;
      case BACKWARDS:
        io.runVolts(-6);
        break;
      case RUN:
        // runVelocity(RPMSetpoint);
        runVelocity(tunableRPMSetpoint.get());
    }
    if (fuelCountDebouncer.calculate(inputs.laserMeasurementInches < fuelDistance)) {
      launchCount++;
    }
  }

  /** Run closed loop at the specified velocity. */
  private void runVelocity(double velocityRPM) {
    boolean inToleranceForTorqueControl =
        Math.abs(inputs.velocityRotsPerMin - velocityRPM) <= torqueCurrentControlTolerance.get();
    boolean torqueCurrentControl = torqueCurrentDebouncer.calculate(inToleranceForTorqueControl);
    atGoal = atGoalDebouncer.calculate(inToleranceForTorqueControl);

    // if (!torqueCurrentControl && lastTorqueCurrentControl) {
    //   launchCount++;
    // }
    lastTorqueCurrentControl = torqueCurrentControl;

    if (inputs.velocityRotsPerMin < velocityRPM + 300) {
      if (torqueCurrentControl) {
        io.runTorqueControl();
      } else {
        io.runDutyCycle();
      }
    } else {
      io.runSlowDutyCycle(velocityRPM);
    }

    // sean's pid + torque control solution
    // if(!torqueCurrentControl) {
    //   io.runDutyCycle();
    // } else if(velocityRPM - inputs.velocityRotsPerMin >= 10) {
    //   io.runTorqueControl();
    // } else {
    //   io.runTorqueControl(velocityRPM);
    // }
  }

  public Command setState(KickerState state) {
    return run(() -> this.state = state).withName("Set " + state);
  }

  public boolean atGoal() {
    return atGoal;
  }
}
