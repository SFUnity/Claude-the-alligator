package frc.robot.subsystems.intakePivot;

import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

public class IntakePivotConstants {
  public static final int pivotID = 13;

  public static final LoggedTunableNumber simkP = new LoggedTunableNumber("Intake/simkP", 0.1);

  public static final boolean pivotInverted = true;

  public static final double minAngleRads = Units.degreesToRadians(10);
  public static final double maxAngleRads = Units.degreesToRadians(135);
  public static final double armLengthMeters = Units.inchesToMeters(15);

  public static final double gearRatio = 66 + 2 / 3;

  // In degrees
  public static final LoggedTunableNumber loweredJorkAngle =
      new LoggedTunableNumber("Intake/lowJorkAngle", 65); // 100
  public static final LoggedTunableNumber raisedJorkAngle =
      new LoggedTunableNumber("Intake/raisedJorkAngle", 50); // 0
  public static final LoggedTunableNumber loweredAngle =
      new LoggedTunableNumber("Intake/loweredAngle", 100);
  public static final LoggedTunableNumber raisedAngle =
      new LoggedTunableNumber("Intake/raisedAngle", Units.rotationsToDegrees(0.0));

  public static final LoggedTunableNumber jorkTolerance =
      new LoggedTunableNumber("Intake/jorkTolerance", 4);
  public static final LoggedTunableNumber isDownTolerance =
      new LoggedTunableNumber("Intake/isDownTolerance", 5);

  public static final LoggedTunableNumber jorkUpVoltage =
      new LoggedTunableNumber("Intake/jorkUpVoltage", -2); // -1.25
  public static final LoggedTunableNumber jorkDownVoltage =
      new LoggedTunableNumber("Intake/jorkDownVoltage", 1.5); // 3
  public static final LoggedTunableNumber extraLoweringVoltage =
      new LoggedTunableNumber("Intake/extraLoweringVoltage", 2);

  public static final LoggedTunableNumber zeroedAngle =
      new LoggedTunableNumber("Intake/zeroedAngle", -10);
}
