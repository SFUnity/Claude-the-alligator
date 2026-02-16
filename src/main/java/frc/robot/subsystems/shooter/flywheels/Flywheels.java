package frc.robot.subsystems.shooter.flywheels;

import static frc.robot.subsystems.shooter.flywheels.FlywheelsConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GeneralUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheels extends SubsystemBase {
  private final FlywheelsIO io;
  private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

  private Debouncer torqueCurrentDebouncer =
      new Debouncer(torqueCurrentDebounce.get(), DebounceType.kFalling);
  private Debouncer atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);
  private boolean lastTorqueCurrentControl = false;

  @AutoLogOutput(key = "Subsystems/Shooter/Flywheels/LaunchCount")
  private long launchCount = 0;

  private double setpointVelocity = 0;

  private boolean ready = false;

  public Flywheels(FlywheelsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Flywheels", inputs);
    GeneralUtil.logSubsystem(this, "Shooter/Flywheels");

    if (ready) {
      runVelocity(setpointVelocity);
    } else {
      runVelocity(readyRPMSetpoint.get());
    }
  }

  /** Run closed loop at the specified velocity. */
  private void runVelocity(double velocityRPM) {
    Logger.recordOutput("Flywheels/goal", velocityRPM);
    boolean inTolerance = (velocityRPM - inputs.velocityRotsPerMin <= flywheelTolerance.get());
    Logger.recordOutput("Flywheels/tolerance", inTolerance);
    boolean torqueCurrentControl = torqueCurrentDebouncer.calculate(inTolerance);
    boolean atGoal = atGoalDebouncer.calculate(inTolerance);
    Logger.recordOutput("Flywheels/atGoal", atGoal);

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

  public void setVelocity(double rpm) {
    setpointVelocity = rpm;
  }

  public void setIsShooting(boolean ready) {
    this.ready = ready;
  }

  public boolean atGoal() {
    return Math.abs(inputs.velocityRotsPerMin - setpointVelocity) < flywheelTolerance.get();
  }
}
