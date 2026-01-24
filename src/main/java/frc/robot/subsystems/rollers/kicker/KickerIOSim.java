package frc.robot.subsystems.rollers.kicker;

public class KickerIOSim implements KickerIO {
  private double appliedVolts = 0.0;
  private double currentAmps = 0.0;

  public KickerIOSim() {}

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = currentAmps;
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
