package frc.robot.subsystems.rollers.spindexer;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.rollers.spindexer.SpindexerIO;

public class SpindexerIOSim extends GenericRollerIOSim implements SpindexerIO {
    private static final DCMotor motorModel = DCMotor.getKrakenX60Foc(1);
    private static final double reduction = (1.0 / 1.0);
    private static final double moi = 0.001;

  public IndexerIOSim() {
    super(motorModel, reduction, moi);
  }
    
}
