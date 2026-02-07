package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShooterUtil {

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      double turretVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed) {}

  public static LaunchingParameters getLaunchingParameters(
      Pose3d targetPose, Pose2d robotPose, ChassisSpeeds robotVelocity) {
    LaunchingParameters params = new LaunchingParameters(false, null, 0, 0, 0, 0);
    return params;
  }
}
