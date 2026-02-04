package frc.robot.subsystems.shooter.hood;

public class HoodIOSim implements HoodIO {
  private double angle;

  public HoodIOSim() {}

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.positionDeg = angle;
  }

  @Override
  public void setPosition(double positionDeg) {
    angle = positionDeg;
  }
}
