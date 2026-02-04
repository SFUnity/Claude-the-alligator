package frc.robot.subsystems.shooter.flywheels;

import frc.robot.util.LoggedTunableNumber;

public class FlywheelsConstants {
  public static int leaderID = 0;
  public static int followID = 0;

  public static LoggedTunableNumber motionMagicCruiseVelocity =
      new LoggedTunableNumber("Flywheels/MotionMagic/CruiseVelocity", 80); // rps
  public static LoggedTunableNumber motionMagicAcceleration =
      new LoggedTunableNumber("Flywheels/MotionMagic/Acceleration", 180);
  public static LoggedTunableNumber motionMagicJerk =
      new LoggedTunableNumber("Flywheels/MotionMagic/Jerk", 1600);

  public static LoggedTunableNumber readyVolts = new LoggedTunableNumber("Flywheels/readyVolts", 6);

  public static LoggedTunableNumber shooterVelocity =
      new LoggedTunableNumber("Flywheels/shootVelocity", 2000);

  public static LoggedTunableNumber kV = new LoggedTunableNumber("Flywheels/kV", 0);
  public static LoggedTunableNumber kA = new LoggedTunableNumber("Flywheels/kA", 0);
  public static LoggedTunableNumber kP = new LoggedTunableNumber("Flywheels/kP", 0);
  public static LoggedTunableNumber kD = new LoggedTunableNumber("Flywheels/kD", 0);

  public static LoggedTunableNumber flywheelTolerance =
      new LoggedTunableNumber("Flywheels/tolerance", 5); // TODO tune
}
