package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Newton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PoseManager;

public class ShooterUtil {

  private final LoggedTunableNumber phaseDelay = new LoggedTunableNumber("Shooter/PhaseDelay", 0.03);
  private final PoseManager poseManager;

  public ShooterUtil(PoseManager poseManager) {
    this.poseManager = poseManager;
  }

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      double turretVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed) {}

  public LaunchingParameters getLaunchingParameters(
      Pose3d targetPose) {
    Pose2d robotPose = poseManager.getPose();
    Twist2d robotVelocity = poseManager.getRobotVelocity();
    robotPose = robotPose.exp(new Twist2d(robotVelocity.dx * phaseDelay.get(), robotVelocity.dy * phaseDelay.get(), 0));
    LaunchingParameters params = new LaunchingParameters(false, null, 0, 0, 0, 0);
    return params;
  }
}
