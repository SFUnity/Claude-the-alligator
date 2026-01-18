package frc.robot.subsystems.rollers;

public class GenericRollerSystemIOSim implements GenericRollerSystemIO {
  private double appliedVolts = 0.0;
  private double currentAmps = 0.0;

  public GenericRollerSystemIOSim() {}

  @Override
  public void updateInputs(GenericRollerInputs inputs) {
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
