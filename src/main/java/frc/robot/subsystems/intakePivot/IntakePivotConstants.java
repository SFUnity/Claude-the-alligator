package frc.robot.subsystems.intakePivot;

import static frc.robot.Constants.*;

import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class IntakePivotConstants {
  // ! Must not be in tuning mode for dashboard switching to work
  public static final LoggedNetworkBoolean groundAlgae =
      new LoggedNetworkBoolean("Ground Algae", false);

  public static final int rollersID = 18;
  public static final int pivotID = 15;

  public static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Intake/kP", currentMode == Mode.SIM ? 0.1 : 0.028);

  public static final boolean pivotInverted = true;
  public static final boolean rollersInverted = false;
  public static final double pivotPositionFactor = 2.9;

  public static final double minAngleRads = Units.degreesToRadians(10);
  public static final double maxAngleRads = Units.degreesToRadians(135);
  public static final double armLengthMeters = Units.inchesToMeters(15);
  public static final double intakePIDMinInput = 0;
  public static final double intakePIDMaxInput = 1 * 360;

  public static final LoggedTunableNumber algaeVoltageThreshold =
      new LoggedTunableNumber("Intake/algaeVoltageThreshold", .5);

  // In rotations
  public static final LoggedTunableNumber loweredAngle =
      new LoggedTunableNumber("Intake/loweredAngle", 20);
  public static final LoggedTunableNumber raisedAngle =
      new LoggedTunableNumber("Intake/raisedAngle", 0);
}
