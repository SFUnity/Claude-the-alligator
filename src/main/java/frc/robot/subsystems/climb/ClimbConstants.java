package frc.robot.subsystems.climb;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class ClimbConstants {
  // TODO remove these two bc they'll always be zero
  // Reason: in a high torque setup (ie big gear ratio) like what's on the climber we don't need to
  // use anything besides P because there is minimal oscillation
  static final double kI = 0;
  static final double kD = 0;

  // TODO change to feet. Meters is for math purposes but we CAD and do most other stuff in feet
  static final double upMeters = 1.0;
  static final double downMeters = 0.0;

  public static final double elevatorMassKg = Units.lbsToKilograms(15); // Not the true numbers
  public static final double drumRadiusMeters = Units.inchesToMeters(1.4); // Not the true numbers
  public static final double minHeightInches = 0; // Not the true numbers
  public static final double maxHeightInches = 23.1; // Not the true numbers
  // TODO assuming we're sticking to length measurement make sure you convert from rotations to
  // length somewhere using circumference.
  public static final double gearRatio = 9; // Not the true numbers

  public static final LoggedTunableNumber kP;

  // TODO use keys with the corect name
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
