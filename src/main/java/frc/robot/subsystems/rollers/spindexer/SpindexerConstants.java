package frc.robot.subsystems.rollers.spindexer;

import frc.robot.util.LoggedTunableNumber;

public class SpindexerConstants {
  public static final int talonID = 0;

  public static final LoggedTunableNumber spindexerSpeedVolts =
      new LoggedTunableNumber("Spindexer/rollerSpeed", 2);
}
