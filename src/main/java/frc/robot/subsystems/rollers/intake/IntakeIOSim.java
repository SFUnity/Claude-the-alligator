package frc.robot.subsystems.rollers.intake;

public class IntakeIOSim implements IntakeIO {
  private double appliedVolts = 0.0;
  private double currentAmps = 0.0;

  public IntakeIOSim() {}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
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
