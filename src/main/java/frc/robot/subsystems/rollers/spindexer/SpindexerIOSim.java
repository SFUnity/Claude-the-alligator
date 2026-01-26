package frc.robot.subsystems.rollers.spindexer;

import frc.robot.subsystems.rollers.spindexer.SpindexerIO.SpindexerIOInputs;

public class SpindexerIOSim implements SpindexerIO {

  private double appliedVolts = 0;

  public SpindexerIOSim() {}

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
  }

  @Override
  public void runVolts(double voltage) {
    appliedVolts = voltage;
  }

  @Override
  public void stop() {
    appliedVolts = 0;
  }
}
