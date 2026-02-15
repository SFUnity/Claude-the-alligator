package frc.robot.subsystems.climb;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class ClimbConstants {

  // TODO change to feet. Meters is for math purposes but we CAD and do most other stuff in feet
  static final double upMeters = Units.inchesToMeters(8.0);
  static final double downMeters = 0.0;

  public static final double elevatorMassKg = Units.lbsToKilograms(15); // Not the true numbers
  public static final double drumRadiusMeters = Units.inchesToMeters(1.4); // Not the true numbers
  public static final double minHeightMeters = 0; // Not the true numbers
  public static final double maxHeightMeters = 23.1; // Not the true numbers

  public static final double gearRatio = 9; // Not the true numbers

  public static final LoggedTunableNumber kP;
  public static final LoggedTunableNumber kG;
  // TODO use keys with the corect name
  static {
    switch (Constants.currentMode) {
      default:
        kP = new LoggedTunableNumber("Climber/kP", 0.028);
        kG = new LoggedTunableNumber("Climber/kG", 0.028);
        break;
      case SIM:
        kP = new LoggedTunableNumber("Climber/simkP", 100);
        kG = new LoggedTunableNumber("Climber/simkG", 0.337525);
        break;
    }
  }
}
