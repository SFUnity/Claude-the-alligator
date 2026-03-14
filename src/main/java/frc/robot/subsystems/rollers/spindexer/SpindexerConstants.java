package frc.robot.subsystems.rollers.spindexer;

import frc.robot.util.LoggedTunableNumber;

public class SpindexerConstants {
  public static final int talonID = 2;

  public static final LoggedTunableNumber spindexerSpeedVolts =
      new LoggedTunableNumber("Spindexer/spindexerSpeedVolts", 6.0);
  public static final LoggedTunableNumber slowSpindexerSpeedVolts =
      new LoggedTunableNumber("Spindexer/slowSpindexerSpeedVolts", 3.0);
}
