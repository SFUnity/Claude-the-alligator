package frc.robot.subsystems.rollers.kicker;

import frc.robot.util.LoggedTunableNumber;

public class KickerConstants {
  public static final int kickerMotorID = 2;
  public static final boolean inverted = false;
  public static final int positionFactor = 3;

  public static final double spinupTime = 0.25;
  public static final LoggedTunableNumber kV = new LoggedTunableNumber("Kicker/kV", 0);
  public static final LoggedTunableNumber kA = new LoggedTunableNumber("Kicker/kA", 0);
  public static final LoggedTunableNumber kP = new LoggedTunableNumber("Kicker/kP", 0);
  public static final LoggedTunableNumber kD = new LoggedTunableNumber("Kicker/kD", 0);
  public static final LoggedTunableNumber setpointVelocity = new LoggedTunableNumber("Kicker/rpm",1000);

  public static final LoggedTunableNumber kickerSpeedVolts =
      new LoggedTunableNumber("Kicker/rollerSpeed", 2);
}
