package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class ShooterConstants {
  public static final double Gravity = 9.81;
  public static final double ShooterMaxRPM = 3500;
  public static final double ShooterMinAngle = 45;
  public static final double ShooterMaxAngle = 75;
  public static final double WheelRadius = 0.0508;
  public static final double TopBottomFlywheelRatio = 1.3;

  public static final Transform3d turretCenter =
      new Transform3d(0.121, 0.133, 0.331, Rotation3d.kZero);
  public static final Translation3d hoodOffset = new Translation3d(0.038, 0, 0.197);
}
