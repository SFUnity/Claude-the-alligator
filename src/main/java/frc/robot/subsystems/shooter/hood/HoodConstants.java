package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

public class HoodConstants {
  public static final int hoodMotorID = 12;
  public static final double gearRatio = 23f * 26f / 12;

  public static final double minPositionDegs = 14.06;
  public static final double maxPositionDegs = 46.333;
  public static final double positionBufferDegs = 1;

  public static final double angleTolerance = 0.5;
  public static final double minAngleRads = Units.degreesToRadians(minPositionDegs);
  public static final double maxAngleRads = Units.degreesToRadians(maxPositionDegs);
  public static final double armLengthMeters = Units.inchesToMeters(15);
  public static final double intakePIDMinInput = 0;
  public static final double intakePIDMaxInput = 1 * 360;

  public static LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP", 550);
  public static LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD", 3);
  public static LoggedTunableNumber statorCurrentTolerance =
      new LoggedTunableNumber("Hood/StatorCurrentTolerance", 4);
  public static LoggedTunableNumber statorCurrentDebounce =
      new LoggedTunableNumber("Hood/StatorCurrentDebounce", 0.2);
}
