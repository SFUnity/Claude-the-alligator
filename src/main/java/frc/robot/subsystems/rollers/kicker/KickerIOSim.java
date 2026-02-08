package frc.robot.subsystems.rollers.kicker;

import static frc.robot.subsystems.rollers.kicker.KickerConstants.*;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.LoggedTunableNumber;

public class KickerIOSim implements KickerIO {
  private double appliedVolts = 0.0;
  private double currentAmps = 0.0;
  private final PIDController controller;

  public KickerIOSim() {
    controller = new PIDController(kP.get(), 0.0, 0.0);
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = currentAmps;
    LoggedTunableNumber.ifChanged(hashCode(), () -> controller.setP(kP.get()), kP);
  }

  @Override
  public void stop() {
    appliedVolts = 0;
  }

  @Override
  public void runVolts(double volts) {
    appliedVolts = volts;
  }
}
