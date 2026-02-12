package frc.robot.subsystems.shooter.flywheels;

import frc.robot.util.LoggedTunableNumber;

public class FlywheelsConstants {
  public static final int leaderID = 0;
  public static final int followID = 0;

  public static final LoggedTunableNumber readyRPMSetpoint =
      new LoggedTunableNumber("Flywheels/readyRPM", 100);
  public static final LoggedTunableNumber ballShotSetpointOffset =
      new LoggedTunableNumber("Flywheels/ballShotSetpointOffset", 100);

  public static final LoggedTunableNumber torqueCurrentTolerance =
      new LoggedTunableNumber("Flywheel/TorqueCurrentControlTolerance", 20.0);
  public static final LoggedTunableNumber torqueCurrentDebounce =
      new LoggedTunableNumber("Flywheel/TorqueCurrentControlDebounce", 0.025);
  public static final LoggedTunableNumber atGoalDebounce =
      new LoggedTunableNumber("Flywheel/AtGoalDebounce", 0.2);

  public static final LoggedTunableNumber flywheelTolerance =
      new LoggedTunableNumber("Flywheels/tolerance", 5); // TODO tune
}
