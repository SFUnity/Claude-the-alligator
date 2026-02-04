package frc.robot.subsystems.rollers.intakerollers;

public class IntakeRollersIOSim implements IntakeRollersIO {
  private double appliedVolts = 0.0;
  private double currentAmps = 0.0;

  public IntakeRollersIOSim() {}

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {
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
