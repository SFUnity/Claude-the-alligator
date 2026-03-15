package frc.robot.subsystems.rollers.kicker;

import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

public class KickerConstants {
  public static final int kickerMotorID = 7;
  public static final int laserCANID = 6;
  public static final boolean inverted = false;
  public static final int positionFactor = 3;

  public static final double spinupTime = 0.25;
  public static final LoggedTunableNumber tunableRPMSetpoint =
      new LoggedTunableNumber(
          "Kicker/RPMSetpoint", 6000); // maybe issue when more than flywheelspeed x 2
  public static double RPMSetpoint = tunableRPMSetpoint.get();

  public static final LoggedTunableNumber torqueCurrentControlTolerance =
      new LoggedTunableNumber(
          "Kicker/TorqueCurrentControlTolerance", Units.radiansPerSecondToRotationsPerMinute(20.0));
  public static final LoggedTunableNumber torqueCurrentControlDebounce =
      new LoggedTunableNumber("Kicker/TorqueCurrentControlDebounce", 0.025);
  public static final LoggedTunableNumber atGoalDebounce =
      new LoggedTunableNumber("Kicker/AtGoalDebounce", 0.2);

  public static final LoggedTunableNumber kickerSpeedVolts =
      new LoggedTunableNumber("Kicker/rollerSpeed", 2);

  public static final double maxRPM = 6000;
}
