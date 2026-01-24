package frc.robot.subsystems.rollers;

public class GenericRollerIOSim implements GenericRollerIO {
  private double appliedVolts = 0.0;
  private double currentAmps = 0.0;

  public GenericRollerIOSim() {}

  @Override
  public void updateInputs(GenericRollerIOInputs inputs) {
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
