package frc.robot.subsystems.shooter.hood;

import frc.robot.util.LoggedTunableNumber;

public class HoodConstants {
  public static final int hoodMotorID = 0;

  public static final double angleTolerance = 2;

  public LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP", 0);
  public LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD", 0);
}
