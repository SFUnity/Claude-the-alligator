package frc.robot.subsystems.shooter.flywheels;

import frc.robot.util.LoggedTunableNumber;

public class FlywheelsConstants {
  public static final int leaderID = 10;
  public static final int followID = 11;

  public static final double gearRatio = 3 / 5; // step up so less than one

  public static final LoggedTunableNumber torqueCurrentControlTolerance =
      new LoggedTunableNumber("Flywheels/TorqueCurrentControlTolerance", 50);
  public static final LoggedTunableNumber torqueCurrentControlDebounce =
      new LoggedTunableNumber("Flywheels/TorqueCurrentControlDebounce", 0.03);
  public static final LoggedTunableNumber atGoalDebounce =
      new LoggedTunableNumber("Flywheels/AtGoalDebounce", 0.2);
}
