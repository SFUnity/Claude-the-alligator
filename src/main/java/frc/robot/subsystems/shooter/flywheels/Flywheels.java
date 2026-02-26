package frc.robot.subsystems.shooter.flywheels;

import static frc.robot.subsystems.shooter.flywheels.FlywheelsConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GeneralUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheels extends SubsystemBase {
  private final FlywheelsIO io;
  private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

  private Debouncer torqueCurrentDebouncer =
      new Debouncer(torqueCurrentControlDebounce.get(), DebounceType.kFalling);
  private Debouncer atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);
  private boolean lastTorqueCurrentControl = false;

  private double RPMSetpoint = 0;

  private boolean torqueCurrentControl = false;
  private boolean atGoal = false;

  @AutoLogOutput(key = "Shooter/Flywheels/LaunchCount")
  private long launchCount = 0;

  public enum FlywheelsState {
    RUN,
    STOP,
    IDLE,
    VOLTS
  }

  private FlywheelsState state = FlywheelsState.STOP;

  public Flywheels(FlywheelsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Flywheels", inputs);
    GeneralUtil.logSubsystem(this, "Shooter/Flywheels");

    Logger.recordOutput("Shooter/Flywheels/atGoal", atGoal);
    Logger.recordOutput("Shooter/Flywheels/torque", torqueCurrentControl);
    Logger.recordOutput("Shooter/Flywheels/state", state.toString());

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
      case IDLE:
        runVelocity(readyRPMSetpoint.get());
        break;
      case RUN:
        runVelocity(RPMSetpoint);
      case VOLTS:
        io.runVolts(2);
    }
  }

  /** Run closed loop at the specified velocity. */
  private void runVelocity(double velocityRPM) {
    Logger.recordOutput("Shooter/Flywheels/goalRPM", velocityRPM);
    boolean inToleranceForTorqueControl =
        Math.abs(inputs.velocityRotsPerMin - velocityRPM) <= torqueCurrentControlTolerance.get();
    boolean torqueCurrentControl = torqueCurrentDebouncer.calculate(inToleranceForTorqueControl);
    atGoal = atGoalDebouncer.calculate(inToleranceForTorqueControl);

    if (!torqueCurrentControl && lastTorqueCurrentControl) {
      launchCount++;
    }
    lastTorqueCurrentControl = torqueCurrentControl;

    if (inputs.velocityRotsPerMin < velocityRPM) {
      if (torqueCurrentControl) {
        io.runTorqueControl();
      } else {
        io.runDutyCycle();
      }
    } else {
      // io.runSlowDutyCycle();
      io.stop();
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

  public void setIsShooting(boolean isShooting) {
    if (state != FlywheelsState.VOLTS) {
      setState(isShooting ? FlywheelsState.RUN : FlywheelsState.IDLE);
    }
  }

  public Command stop() {
    return Commands.runOnce(() -> setState(FlywheelsState.STOP));
  }

  public void setState(FlywheelsState state) {
    this.state = state;
  }

  public Command setVelocity(double velocityRPM) {
    return runOnce(
        () -> {
          this.RPMSetpoint = velocityRPM;
        });
  }

  public boolean atGoal() {
    return atGoal;
  }

  public double getVelocityRPM() {
    return inputs.velocityRotsPerMin;
  }
}
