package frc.robot.subsystems.shooter.flywheels;

import frc.robot.util.LoggedTunableNumber;

public class FlywheelsConstants {
  public static final int leaderID = 10;
  public static final int followID = 11;

  public static final double gearRatio = 3 / 5; // step up so less than one

  public static final LoggedTunableNumber readyRPMSetpoint =
      new LoggedTunableNumber("Flywheels/readyRPM", 100);

  public static final LoggedTunableNumber torqueCurrentControlTolerance =
      new LoggedTunableNumber("Flywheels/TorqueCurrentControlTolerance", 20);
  public static final LoggedTunableNumber torqueCurrentControlDebounce =
      new LoggedTunableNumber("Flywheels/TorqueCurrentControlDebounce", 0.025);
  public static final LoggedTunableNumber atGoalDebounce =
      new LoggedTunableNumber("Flywheels/AtGoalDebounce", 0.2);
}
