package frc.robot.subsystems.shooter.flywheels;

import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

public class FlywheelsConstants {
  public static final int leaderID = 5;
  public static final int followID = 6;

  public static final double gearRatio = 2 / 3; // step up so less than one

  public static final LoggedTunableNumber readyRPMSetpoint =
      new LoggedTunableNumber("Flywheels/readyRPM", 100);

  public static final LoggedTunableNumber torqueCurrentControlTolerance =
      new LoggedTunableNumber(
          "Flywheels/TorqueCurrentControlTolerance",
          Units.radiansPerSecondToRotationsPerMinute(20.0));
  public static final LoggedTunableNumber torqueCurrentControlDebounce =
      new LoggedTunableNumber("Flywheels/TorqueCurrentControlDebounce", 0.025);
  public static final LoggedTunableNumber atGoalDebounce =
      new LoggedTunableNumber("Flywheels/AtGoalDebounce", 0.2);
}
