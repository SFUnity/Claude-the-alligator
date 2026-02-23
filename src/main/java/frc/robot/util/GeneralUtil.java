package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public abstract class GeneralUtil {
  public static boolean equalsWithTolerance(double a, double b, double tolerance) {
    return (a - tolerance <= b) && (a + tolerance >= b);
  }

  public static void logSubsystem(SubsystemBase s, String sName) {
    Logger.recordOutput(
        "CurrentCmds/" + sName,
        s.getCurrentCommand() != null ? s.getCurrentCommand().getName() : "none");
  }
}
