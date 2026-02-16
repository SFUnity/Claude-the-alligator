package frc.robot.subsystems.shooter.hood;

import frc.robot.util.LoggedTunableNumber;

public class HoodConstants {
  public static final int hoodMotorID = 7;
  public static final double gearRatio = 73.5;

  public static final double angleTolerance = 2;

  public static LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP", 0);
  public static LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD", 0);
}
