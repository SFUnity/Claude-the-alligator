package frc.robot.subsystems.rollers.kicker;

import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

public class KickerConstants {
  public static final int kickerMotorID = 7;
  public static final int laserCANID = 6;
  public static final boolean inverted = false;
  public static final int positionFactor = 3;

  public static final double spinupTime = 0.25;
  public static final LoggedTunableNumber RPMSetpoint =
      new LoggedTunableNumber("Kicker/RPMSetpoint", Units.radiansPerSecondToRotationsPerMinute(100));

  public static final LoggedTunableNumber torqueCurrentControlTolerance =
      new LoggedTunableNumber("Flywheel/TorqueCurrentControlTolerance", 20.0);
  public static final LoggedTunableNumber torqueCurrentControlDebounce =
      new LoggedTunableNumber("Flywheel/TorqueCurrentControlDebounce", 0.025);
  public static final LoggedTunableNumber atGoalDebounce =
      new LoggedTunableNumber("Flywheel/AtGoalDebounce", 0.2);

  public static final LoggedTunableNumber kickerSpeedVolts =
      new LoggedTunableNumber("Kicker/rollerSpeed", 2);
}
