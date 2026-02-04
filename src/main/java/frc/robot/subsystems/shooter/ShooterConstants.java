package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class ShooterConstants {
  public static final double Gravity = 9.81;
  public static final double ShooterMaxRPM = 3500;
  public static final double ShooterMinAngle = 45;
  public static final double ShooterMaxAngle = 75;

  public static final Pose3d turretCenter = new Pose3d(0.121, 0.133, 0.331, new Rotation3d());
  public static final Translation3d hoodOffset = new Translation3d(0.038, 0, 0.197);
}
