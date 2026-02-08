package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.util.GeomUtil.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.FieldConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PoseManager;

public class ShooterUtil {

  private final LoggedTunableNumber phaseDelay =
      new LoggedTunableNumber("Shooter/PhaseDelay", 0.03);
  private final PoseManager poseManager;

  private final InterpolatingDoubleTreeMap launchHoodAngleMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap launchFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  public ShooterUtil(PoseManager poseManager) {
    this.poseManager = poseManager;
  }

  public record LaunchingParameters(
      boolean isValid,
      double turretAngle,
      double turretVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed) {}

  public LaunchingParameters getScoringParameters() {
    Pose2d robotPose = poseManager.getPose();
    Twist2d robotVelocity = poseManager.getRobotVelocity();
    Translation2d targetPose =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    robotPose =
        robotPose.exp(
            new Twist2d(
                robotVelocity.dx * phaseDelay.get(), robotVelocity.dy * phaseDelay.get(), 0));

    Pose2d turretPosition =
        robotPose.transformBy(
            new Transform2d(
                turretCenter.getTranslation().toTranslation2d(),
                turretCenter.getRotation().toRotation2d()));
    double turretToTargetDistance = targetPose.getDistance(turretPosition.getTranslation());

    Twist2d fieldRelativeRobotVelocity = poseManager.getFieldVelocity();
    double robotAngle = robotPose.getRotation().getRadians();
    double turretVelocityX =
        fieldRelativeRobotVelocity.dx
            + fieldRelativeRobotVelocity.dtheta
                * (turretCenter.getY() * Math.cos(robotAngle)
                    - turretCenter.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        fieldRelativeRobotVelocity.dy
            + fieldRelativeRobotVelocity.dtheta
                * (turretCenter.getX() * Math.cos(robotAngle)
                    - turretCenter.getY() * Math.sin(robotAngle));

    double timeOfFlight;
    Pose2d lookeaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookeaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = targetPose.getDistance(lookeaheadPose.getTranslation());
    }

    LaunchingParameters params = new LaunchingParameters(false, 0, 0, 0, 0, 0);
    return params;
  }
}
