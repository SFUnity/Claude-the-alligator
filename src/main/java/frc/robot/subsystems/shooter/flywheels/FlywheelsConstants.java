package frc.robot.subsystems.shooter.flywheels;

import frc.robot.util.LoggedTunableNumber;

public class FlywheelsConstants {
  public static final int leaderID = 0;
  public static final int followID = 0;

  public static final LoggedTunableNumber motionMagicCruiseVelocity =
      new LoggedTunableNumber("Flywheels/MotionMagic/CruiseVelocity", 80); // rps
  public static final LoggedTunableNumber motionMagicAcceleration =
      new LoggedTunableNumber("Flywheels/MotionMagic/Acceleration", 180);
  public static final LoggedTunableNumber motionMagicJerk =
      new LoggedTunableNumber("Flywheels/MotionMagic/Jerk", 1600);

  public static final LoggedTunableNumber readyVolts =
      new LoggedTunableNumber("Flywheels/readyVolts", 6);

  public static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheels/kV", 0);
  public static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheels/kA", 0);
  public static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheels/kP", 0);
  public static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheels/kD", 0);

  public static final LoggedTunableNumber flywheelTolerance =
      new LoggedTunableNumber("Flywheels/tolerance", 5); // TODO tune
}
