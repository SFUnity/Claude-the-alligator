package frc.robot.subsystems.shooter;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.util.GeomUtil.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.FieldConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.PoseManager;
import java.util.LinkedList;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

public class ShooterUtil {

  private final double phaseDelay = loopPeriodSecs;
  private final PoseManager poseManager;

  private final InterpolatingDoubleTreeMap scoreRealHoodAngleMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap scoreRealFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap scoreRealTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  private final InterpolatingDoubleTreeMap feedRealHoodAngleMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap feedRealFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap feedRealTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  private final InterpolatingDoubleTreeMap scoreSimHoodAngleMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap scoreSimFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap scoreSimTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  private final InterpolatingDoubleTreeMap feedSimHoodAngleMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap feedSimFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap feedSimTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  private double turretAngle;
  private Queue<Double> turretAngles = new LinkedList<>();
  private double hoodAngle;
  private Queue<Double> hoodAngles = new LinkedList<>();
  private double turretVelocity;
  private double hoodVelocity;

  private double minDist = 0; // todo change
  private double maxDist = 1000;

  private final int sampleCount = 50;

  private final LinearFilter turretAngleFilter = LinearFilter.movingAverage(sampleCount);
  private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage(sampleCount);

  public ShooterUtil(PoseManager poseManager) {
    this.poseManager = poseManager;

    // REAL
    feedRealFlywheelSpeedMap.put(1.376, 1075.0);
    feedRealHoodAngleMap.put(1.376, 15.0);
    feedRealTimeOfFlightMap.put(1.376, 0.99);

    scoreRealFlywheelSpeedMap.put(1.488, 1075.0);
    scoreRealHoodAngleMap.put(1.488, 15.0);
    scoreRealTimeOfFlightMap.put(1.488, 0.0);

    scoreRealFlywheelSpeedMap.put(2.462, 1150.0);
    scoreRealHoodAngleMap.put(2.462, 19.0);
    // scoreRealTimeOfFlightMap.put(, );

    scoreRealFlywheelSpeedMap.put(3.445, 1200.0);
    scoreRealHoodAngleMap.put(3.445, 24.0);

    scoreRealFlywheelSpeedMap.put(4.513, 1250.0);
    scoreRealHoodAngleMap.put(4.513, 28.0);

    scoreRealFlywheelSpeedMap.put(5.419, 1410.0);
    scoreRealHoodAngleMap.put(5.419, 32.5);

    // scoreRealFlywheelSpeedMap.put(, );
    // scoreRealHoodAngleMap.put(, );
  }

  public record LaunchingParameters(
      boolean isValid,
      double turretAngle,
      double turretVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed) {}

  public LaunchingParameters getLaunchingParameters(boolean isScoring, boolean isReal) {
    InterpolatingDoubleTreeMap hoodAngleMap;
    InterpolatingDoubleTreeMap flywheelSpeedMap;
    InterpolatingDoubleTreeMap timeOfFlightMap;

    if (isScoring) {
      if (isReal) {
        hoodAngleMap = scoreRealHoodAngleMap;
        flywheelSpeedMap = scoreRealFlywheelSpeedMap;
        timeOfFlightMap = scoreRealTimeOfFlightMap;
      } else {
        hoodAngleMap = scoreSimHoodAngleMap;
        flywheelSpeedMap = scoreSimFlywheelSpeedMap;
        timeOfFlightMap = scoreSimTimeOfFlightMap;
      }
    } else {
      if (isReal) {
        hoodAngleMap = feedRealHoodAngleMap;
        flywheelSpeedMap = feedRealFlywheelSpeedMap;
        timeOfFlightMap = feedRealTimeOfFlightMap;
      } else {
        hoodAngleMap = feedSimHoodAngleMap;
        flywheelSpeedMap = feedSimFlywheelSpeedMap;
        timeOfFlightMap = feedSimTimeOfFlightMap;
      }
    }

    Pose2d robotPose = poseManager.getPose();
    Twist2d robotVelocity = poseManager.getRobotVelocity();
    robotPose =
        robotPose.exp(
            new Twist2d(
                robotVelocity.dx * phaseDelay,
                robotVelocity.dy * phaseDelay,
                robotVelocity.dtheta * phaseDelay));

    Pose2d turretPosition =
        robotPose.transformBy(
            new Transform2d(
                turretCenter.getTranslation().toTranslation2d(),
                turretCenter.getRotation().toRotation2d()));
    Translation2d targetPose;
    if (isScoring) {
      targetPose = // TODO change if feeding and not scoring
          AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    } else {
      if (turretPosition.getY() > FieldConstants.fieldWidth / 2) {
        targetPose =
            AllianceFlipUtil.apply(
                new Translation2d(AllianceFlipUtil.applyX(0), FieldConstants.fieldWidth - 0.5));
      } else {
        targetPose = AllianceFlipUtil.apply(new Translation2d(AllianceFlipUtil.applyX(0), 0.5));
      }
    }
    Logger.recordOutput("Shooter/Turret/Target", new Pose2d(targetPose, new Rotation2d()));

    Logger.recordOutput("Shooter/Turret/CurrentTurretPose", turretPosition);
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
      // timeOfFlight = 0.5; // TODO: replace with actual time of flight calculation
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookeaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = targetPose.getDistance(lookeaheadPose.getTranslation());
    }
    // if (lookeaheadPose.getX() < 0)
    //   lookeaheadPose = new Pose2d(0, lookeaheadPose.getY(), lookeaheadPose.getRotation());
    // if (lookeaheadPose.getX() > fieldLength)
    //   lookeaheadPose = new Pose2d(fieldLength, lookeaheadPose.getY(),
    // lookeaheadPose.getRotation());
    // if (lookeaheadPose.getY() < 0)
    //   lookeaheadPose = new Pose2d(lookeaheadPose.getX(), 0, lookeaheadPose.getRotation());
    // if (lookeaheadPose.getY() > fieldWidth)
    //   lookeaheadPose = new Pose2d(lookeaheadPose.getX(), fieldWidth,
    // lookeaheadPose.getRotation());

    Logger.recordOutput(
        "Shooter/Turret/LookaheadTurretToTargetDist", lookaheadTurretToTargetDistance);

    Logger.recordOutput("Shooter/Turret/LookaheadTurretPose", lookeaheadPose);

    lookaheadTurretToTargetDistance += 0.25; // shoot to back of hub, not center

    turretAngle =
        targetPose.minus(lookeaheadPose.getTranslation()).getAngle().getDegrees()
            - poseManager.getRotation().getDegrees();
    hoodAngle = hoodAngleMap.get(lookaheadTurretToTargetDistance);

    if (turretAngles.isEmpty()) turretAngles.add(turretAngle);
    if (hoodAngles.isEmpty()) hoodAngles.add(hoodAngle);

    turretAngles.add(turretAngle);
    hoodAngles.add(hoodAngle);

    turretVelocity =
        turretAngleFilter.calculate((turretAngle - turretAngles.remove()) / sampleCount);
    hoodVelocity = hoodAngleFilter.calculate((hoodAngle - hoodAngles.remove()) / sampleCount);

    LaunchingParameters params =
        new LaunchingParameters(
            // minDist < lookaheadTurretToTargetDistance && lookaheadTurretToTargetDistance <
            // maxDist,
            true,
            turretAngle, // doesn't matter
            turretVelocity, // doesn't matter
            scoreRealHoodAngleMap.get(turretToTargetDistance),
            hoodVelocity, // doesn't matter
            scoreRealFlywheelSpeedMap.get(turretToTargetDistance));
    return params;
  }
}
