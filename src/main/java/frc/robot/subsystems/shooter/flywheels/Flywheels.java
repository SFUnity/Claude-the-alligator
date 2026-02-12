package frc.robot.subsystems.shooter.flywheels;

import static frc.robot.subsystems.shooter.flywheels.FlywheelsConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
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
  @AutoLogOutput private long launchCount = 0;
  private double setpointVelocity;

  private boolean ready = false;

  public Flywheels(FlywheelsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheels", inputs);
    GeneralUtil.logSubsystem(this, "Flywheels");

    if (ready) {
      if (inputs.velocityRotsPerMin < setpointVelocity - ballShotSetpointOffset.get()) io.runDutyCycle();
      else if (inputs.velocityRotsPerMin < setpointVelocity) io.runTorqueControl();
    } else {
      if (inputs.velocityRotsPerMin < readyRPMSetpoint.get() - ballShotSetpointOffset.get())
        io.runDutyCycle();
      else if (inputs.velocityRotsPerMin < readyRPMSetpoint.get()) io.runTorqueControl();
    }
  }

  public Command setVelocity(double rpm) {
    return run(() -> setpointVelocity = rpm);
  }

  public Command setReady(boolean ready) {
    return run(() -> this.ready = ready);
  }

  public boolean atGoal() {
    return Math.abs(inputs.velocityRotsPerMin - setpointVelocity) < flywheelTolerance.get();
  }
}
