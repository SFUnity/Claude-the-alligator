package frc.robot.subsystems.shooter;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.util.GeomUtil.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
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
  private final InterpolatingDoubleTreeMap scoreRealKickerSpeedMap =
      new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap scoreRealTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  private final InterpolatingDoubleTreeMap feedRealHoodAngleMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap feedRealFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap feedRealKickerSpeedMap =
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

    // scoreRealFlywheelSpeedMap.put(, );
    // scoreRealHoodAngleMap.put(, );
    // scoreRealTimeOfFlightMap.put(, );

    scoreSimFlywheelSpeedMap.put(1.256, 1200.0);
    scoreSimHoodAngleMap.put(1.256, 14.6);
    scoreSimTimeOfFlightMap.put(1.256, 0.900);

    scoreSimFlywheelSpeedMap.put(1.454, 1200.0);
    scoreSimHoodAngleMap.put(1.454, 18.0);
    scoreSimTimeOfFlightMap.put(1.454, 0.860);

    scoreSimFlywheelSpeedMap.put(1.784, 1200.0);
    scoreSimHoodAngleMap.put(1.784, 25.0);
    scoreSimTimeOfFlightMap.put(1.784, 0.760);

    scoreSimFlywheelSpeedMap.put(1.908, 1200.0);
    scoreSimHoodAngleMap.put(1.908, 28.0);
    scoreSimTimeOfFlightMap.put(1.908, 0.680);

    scoreSimFlywheelSpeedMap.put(2.117, 1300.0);
    scoreSimHoodAngleMap.put(2.117, 17.5);
    scoreSimTimeOfFlightMap.put(2.117, 1.020);

    scoreSimFlywheelSpeedMap.put(2.302, 1300.0);
    scoreSimHoodAngleMap.put(2.302, 24.0);
    scoreSimTimeOfFlightMap.put(2.302, 0.940);

    scoreSimFlywheelSpeedMap.put(2.569, 1400.0);
    scoreSimHoodAngleMap.put(2.569, 26.0);
    scoreSimTimeOfFlightMap.put(2.569, 1.043);

    scoreSimFlywheelSpeedMap.put(2.700, 1350.0);
    scoreSimHoodAngleMap.put(2.700, 26.0);
    scoreSimTimeOfFlightMap.put(2.700, 0.960);

    scoreSimFlywheelSpeedMap.put(3.074, 1500.0);
    scoreSimHoodAngleMap.put(3.074, 20.5);
    scoreSimTimeOfFlightMap.put(3.074, 1.220);

    scoreSimFlywheelSpeedMap.put(3.203, 1350.0);
    scoreSimHoodAngleMap.put(3.203, 33.0);
    scoreSimTimeOfFlightMap.put(3.203, 0.840);

    scoreSimFlywheelSpeedMap.put(3.399, 1400.0);
    scoreSimHoodAngleMap.put(3.399, 35.0);
    scoreSimTimeOfFlightMap.put(3.399, 0.860);

    scoreSimFlywheelSpeedMap.put(3.589, 1450.0);
    scoreSimHoodAngleMap.put(3.589, 35.0);
    scoreSimTimeOfFlightMap.put(3.589, 0.920);

    scoreSimFlywheelSpeedMap.put(3.797, 1450.0);
    scoreSimHoodAngleMap.put(3.797, 35.0);
    scoreSimTimeOfFlightMap.put(3.797, 0.920);

    scoreSimFlywheelSpeedMap.put(4.025, 1500.0);
    scoreSimHoodAngleMap.put(4.025, 40.0);
    scoreSimTimeOfFlightMap.put(4.025, 0.860);

    scoreSimFlywheelSpeedMap.put(4.212, 1500.0);
    scoreSimHoodAngleMap.put(4.212, 35.0);
    scoreSimTimeOfFlightMap.put(4.212, 0.980);

    scoreSimFlywheelSpeedMap.put(4.458, 1500.0);
    scoreSimHoodAngleMap.put(4.458, 33.0);
    scoreSimTimeOfFlightMap.put(4.458, 1.020);

    scoreSimFlywheelSpeedMap.put(4.607, 1600.0);
    scoreSimHoodAngleMap.put(4.607, 38.0);
    scoreSimTimeOfFlightMap.put(4.607, 1.020);

    scoreSimFlywheelSpeedMap.put(4.807, 1600.0);
    scoreSimHoodAngleMap.put(4.807, 37.0);
    scoreSimTimeOfFlightMap.put(4.807, 1.040);

    scoreSimFlywheelSpeedMap.put(5.117, 1600.0);
    scoreSimHoodAngleMap.put(5.117, 35.0);
    scoreSimTimeOfFlightMap.put(5.117, 1.100);

    scoreSimFlywheelSpeedMap.put(5.312, 1650.0);
    scoreSimHoodAngleMap.put(5.312, 38.0);
    scoreSimTimeOfFlightMap.put(5.312, 1.080);

    // Measured at 10th Street
    scoreRealFlywheelSpeedMap.put(0.0, 1101.0);
    scoreRealHoodAngleMap.put(0.0, 15.0);
    scoreRealTimeOfFlightMap.put(0.0, 1.0);

    scoreRealFlywheelSpeedMap.put(0.5, 1101.0);
    scoreRealHoodAngleMap.put(0.5, 17.0);
    scoreRealTimeOfFlightMap.put(0.5, null);

    scoreRealFlywheelSpeedMap.put(0.5, 1100.0);
    scoreRealHoodAngleMap.put(0.5, 17.0);
    scoreRealTimeOfFlightMap.put(0.5, null);
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
    Translation2d targetPose = // TODO change if feeding and not scoring
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    Logger.recordOutput("Shooter/Turret/Target", targetPose);
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
            minDist < lookaheadTurretToTargetDistance && lookaheadTurretToTargetDistance < maxDist,
            turretAngle,
            turretVelocity,
            hoodAngle,
            hoodVelocity,
            flywheelSpeedMap.get(lookaheadTurretToTargetDistance));
    return params;
  }
}
