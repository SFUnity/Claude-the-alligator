package frc.robot.subsystems.rollers.spindexer;

// TODO please remove unused import
import frc.robot.subsystems.rollers.spindexer.SpindexerIO.SpindexerIOInputs;

public class SpindexerIOSim implements SpindexerIO {

  private double appliedVolts = 0;

  public SpindexerIOSim() {}

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
  }

  @Override
  public void run(double voltage) {
    appliedVolts = voltage;
  }

  // TODO you can delete this because it doesn't get used anywhere
  @Override
  public void stop() {
    appliedVolts = 0;
  }
}
