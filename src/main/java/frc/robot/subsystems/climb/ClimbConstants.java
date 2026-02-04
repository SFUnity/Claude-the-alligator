package frc.robot.subsystems.climb;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class ClimbConstants {

  static final double kI = 0;
  static final double kD = 0;

  static final double upMeters = 1.0;
  static final double downMeters = 0.0;

  public static final double elevatorMassKg = Units.lbsToKilograms(15); // Not the true numbers
  public static final double drumRadiusMeters = Units.inchesToMeters(1.4); // Not the true numbers
  public static final double minHeightInches = 0; // Not the true numbers
  public static final double maxHeightInches = 23.1; // Not the true numbers
  public static final double gearRatio = 9; // Not the true numbers

  public static final LoggedTunableNumber kP;

  static {
    switch (Constants.currentMode) {
      default:
        kP = new LoggedTunableNumber("Intake/kP", 0.028);
        break;
      case SIM:
        kP = new LoggedTunableNumber("Intake/simkP", 0.1);
        break;
    }
  }
}
