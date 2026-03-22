package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.LoggedTunableNumber;

public class ShooterConstants {
  public static final double Gravity = 9.81;
  public static final double ShooterMaxRPM = 3500;
  public static final double ShooterMinAngle = 45;
  public static final double ShooterMaxAngle = 75;
  public static final double WheelRadius = 0.0508;
  public static final double TopBottomFlywheelRatio = 1.3;

  public static final Translation3d hoodOffset = new Translation3d(0.038, 0, 0.197);

  // Trench passign stuff
  public static final double trenchRadius = 3f;

  public static final LoggedTunableNumber trenchAvoidenceDebouncerTime =
      new LoggedTunableNumber("Controls/Trench Avoidence/trenchAvoidenceDebouncerTime", 0.2);

  public static final LoggedTunableNumber autoHoodAngle =
      new LoggedTunableNumber("SmartDashboard/TunableNumbers/Shooter/AutoHoodAngle", 26);
  public static final LoggedTunableNumber autoFlywheelVelocity =
      new LoggedTunableNumber("SmartDashboard/TunableNumbers/Shooter/AutoFlywheelVelocity", 1200);
}
