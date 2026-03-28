package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.util.Units;

public class HoodConstants {
  public static final int hoodMotorID = 12;
  public static final double gearRatio = 23f * 26f / 12;

  public static final double minPositionDegs = 14.06;
  public static final double maxPositionDegs = 46.333;
  public static final double positionBufferDegs = 1.5;
  public static final double positionOffsetRots = .1;

  public static final double angleTolerance = 0.5;
  public static final double minAngleRads = Units.degreesToRadians(minPositionDegs);
  public static final double maxAngleRads = Units.degreesToRadians(maxPositionDegs);
  public static final double armLengthMeters = Units.inchesToMeters(15);
}
