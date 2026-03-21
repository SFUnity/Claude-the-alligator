package frc.robot.subsystems.climb;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class ClimbConstants {
  public static final int climbMotorID = 4;

  static final LoggedTunableNumber upMeters =
      new LoggedTunableNumber("Climber/upMeters", Units.inchesToMeters(8.0));
  static final LoggedTunableNumber downMeters = new LoggedTunableNumber("Climber/downMeters", 0);
  static final LoggedTunableNumber upVolts = new LoggedTunableNumber("Climber/upVolts", 1);
  static final LoggedTunableNumber downVolts = new LoggedTunableNumber("Climber/downVolts", -2);

  public static final double loopPeriodSecs = 0.02;

  public static final double elevatorMassKg = Units.lbsToKilograms(15); // Not the true numbers
  public static final double drumRadiusMeters = Units.inchesToMeters(0.75);
  public static final double minHeightMeters = 0; // Not the true numbers
  public static final double maxHeightMeters = 23.1; // Not the true numbers

  public static final double gearRatio = 5;

  public static final LoggedTunableNumber kP;
  public static final LoggedTunableNumber kG;

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
