package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

public class HoodConstants {
  public static final int hoodMotorID = 7;
  public static final double gearRatio = 23 * 26/12; // ~25

  public static final double angleTolerance = 2;
  public static final double minAngleRads = Units.degreesToRadians(14.06);
  public static final double maxAngleRads = Units.degreesToRadians(46.5);
  public static final double armLengthMeters = Units.inchesToMeters(15);
  public static final double intakePIDMinInput = 0;
  public static final double intakePIDMaxInput = 1 * 360;

  // TODO add values
  public static final double minPositionDegs = 14.06;
  public static final double maxPositionDegs = 46.5;

  public static LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP", 0);
  public static LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD", 0);
}
