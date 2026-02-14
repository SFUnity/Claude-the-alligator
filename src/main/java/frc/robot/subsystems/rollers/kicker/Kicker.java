package frc.robot.subsystems.rollers.kicker;

import static edu.wpi.first.wpilibj2.command.Commands.select;
import static frc.robot.subsystems.rollers.kicker.KickerConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.flywheels.FlywheelsIO;
import frc.robot.util.GeneralUtil;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  // public Kicker(KickerIO io) {
  //   this.io = io;
  // }

  // @Override
  // public void periodic() {
  //   io.updateInputs(inputs);
  //   Logger.processInputs("Kicker", inputs);
  //   GeneralUtil.logSubsystem(this, "Kicker");
  // }

  // public Command run() {
  //   return run(() -> io.runVolts(kickerSpeedVolts.get())).withName("runVolts");
  // }

  // public Command stop() {
  //   return run(() -> io.runVolts(0)).withName("stop");
  // }

  // public Command runBack() {
  //   return run(() -> io.runVolts(-kickerSpeedVolts.get())).withName("runVolts");
  // }
  private Debouncer torqueCurrentDebouncer =
      new Debouncer(torqueCurrentDebounce.get(), DebounceType.kFalling);
  private Debouncer atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);
  private boolean lastTorqueCurrentControl = false;

  @AutoLogOutput(key = "Subsystems/Rollers/Kicker/LaunchCount")
  private long launchCount = 0;

  public enum KickerState  {
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

    switch(state){
      case STOP:
      io.runVolts(0);
      break;
      case BACKWARDS:
      io.runVolts(-6);
      break;
      case RUN:
      runVelocity(RPMSetpoint.get());
    }
  }

  /** Run closed loop at the specified velocity. */
  private void runVelocity(double velocityRPM) {
    boolean inTolerance =
        Math.abs(inputs.velocityRotsPerMin - velocityRPM) <= torqueCurrentTolerance.get();
    boolean torqueCurrentControl = torqueCurrentDebouncer.calculate(inTolerance);
    boolean atGoal = atGoalDebouncer.calculate(inTolerance);

    if (!torqueCurrentControl && lastTorqueCurrentControl) {
      launchCount++;
    }
    lastTorqueCurrentControl = torqueCurrentControl;

    if (!atGoal) {
      if (torqueCurrentControl) {
        io.runTorqueControl();
      } else {
        io.runDutyCycle();
      }
    }
  }

  // public Command setVelocity(double rpm) {
  //   return run(() -> setpointVelocity = rpm);
  // }

  // public Command setReady(boolean ready) {
  //   return run(() -> this.ready = ready);
  // }

  public Command setState(KickerState state){
    return run(()-> this.state = state);
  }

  public boolean atGoal() {
    return Math.abs(inputs.velocityRotsPerMin - RPMSetpoint.get()) < kickerTolerance.get();
  }
}
