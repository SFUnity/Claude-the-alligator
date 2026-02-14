package frc.robot.subsystems.rollers.kicker;

import frc.robot.util.LoggedTunableNumber;

public class KickerConstants {
  public static final int kickerMotorID = 2;
  public static final boolean inverted = false;
  public static final int positionFactor = 3;

  public static final double spinupTime = 0.25;
  public static final LoggedTunableNumber torqueCurrentTolerance =
      new LoggedTunableNumber("Flywheel/TorqueCurrentTolerance", 1200.0);
  public static final LoggedTunableNumber torqueCurrentDebounce =
      new LoggedTunableNumber("Flywheel/TorqueCurrentDebounce", 0.025);
  public static final LoggedTunableNumber atGoalDebounce =
      new LoggedTunableNumber("Flywheel/AtGoalDebounce", 0.2);
  public static final LoggedTunableNumber kickerTolerance =
    new LoggedTunableNumber("Kicker/tolerance", 5); // TODO tune



  public static final LoggedTunableNumber kickerSpeedVolts =
      new LoggedTunableNumber("Kicker/rollerSpeed", 2);
}
