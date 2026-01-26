package frc.robot.subsystems.rollers.intake;

import frc.robot.util.LoggedTunableNumber;

public class IntakeConstants {
  public static final int intakeMotorID = 2;
  public static final boolean inverted = false;
  public static final int positionFactor = 3;

  public static final LoggedTunableNumber intakeSpeedVolts =
      new LoggedTunableNumber("Intake/rollerSpeed", 2);
}
