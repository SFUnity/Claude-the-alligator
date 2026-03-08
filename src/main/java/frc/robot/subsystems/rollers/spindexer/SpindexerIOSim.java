package frc.robot.subsystems.rollers.spindexer;

public class SpindexerIOSim implements SpindexerIO {

  private double appliedVolts = 0;
  private double positionRots = 0;

  public SpindexerIOSim() {}

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    // 1 V == 0.1 rot/s
    // loopPeriod is .02s
    // So 1 V applied for 1 loop == 0.002 rot change
    inputs.appliedVolts = appliedVolts;
    inputs.positionRots =
        positionRots += appliedVolts * 0.002; // Simulate position change based on voltage
  }

  @Override
  public void run(double voltage) {
    appliedVolts = voltage;
  }
}
