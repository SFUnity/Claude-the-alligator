package frc.robot.subsystems.climb;

public class ClimbIOSim implements ClimbIO {

  private double appliedVolts = 0;

  public ClimbIOSim() {}

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
  }

  @Override
  public void setPosition(double rotations) {
    appliedVolts = rotations;
  }
}
