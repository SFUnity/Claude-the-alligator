package frc.robot.subsystems.rollers.intakerollers;

import frc.robot.util.LoggedTunableNumber;

public class IntakeRollersConstants {
  public static final int intakeRollersMotorID = 14;
  public static final boolean inverted = false;
  public static final int positionFactor = 3;

  public static final LoggedTunableNumber intakeRollersSpeedVolts =
      new LoggedTunableNumber("rollerSpeed", 6.5);
}
