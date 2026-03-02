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

  // In rotations
  public static final LoggedTunableNumber loweredJorkAngle =
      new LoggedTunableNumber("Intake/lowJorkAngle", .2);
  public static final LoggedTunableNumber raisedJorkAngle =
      new LoggedTunableNumber("Intake/raisedJorkAngle", .1);
  public static final LoggedTunableNumber loweredAngle =
      new LoggedTunableNumber("Intake/loweredAngle", .29);
  public static final LoggedTunableNumber raisedAngle =
      new LoggedTunableNumber("Intake/raisedAngle", 0);
}
