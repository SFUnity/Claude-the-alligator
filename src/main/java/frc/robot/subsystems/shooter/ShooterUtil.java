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
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.PoseManager;
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
  private double hoodAngle;

  private double minDist = 0; // todo change
  private double maxDist = 1000;

  private final int sampleCount = 50;

  private final LinearFilter turretAngleFilter = LinearFilter.movingAverage(sampleCount);
  private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage(sampleCount);

  public ShooterUtil(PoseManager poseManager) {
    this.poseManager = poseManager;

    // FEEDING REAL
    feedRealFlywheelSpeedMap.put(15.25, 3000.0);
    feedRealHoodAngleMap.put(15.25, 49.0);
    feedRealTimeOfFlightMap.put(1.376, 0.99);

    feedRealFlywheelSpeedMap.put(12.0, 2000.0);
    feedRealHoodAngleMap.put(12.0, 45.0);

    feedRealFlywheelSpeedMap.put(9.8, 1500.0);
    feedRealHoodAngleMap.put(9.8, 42.0);

    feedRealFlywheelSpeedMap.put(8.1, 1300.0);
    feedRealHoodAngleMap.put(8.1, 38.0);

    feedRealFlywheelSpeedMap.put(6.5, 1100.0);
    feedRealHoodAngleMap.put(6.5, 35.0);

    // SCORING REAL
    scoreRealFlywheelSpeedMap.put(1.488, 1110.0);
    scoreRealHoodAngleMap.put(1.488, 15.0);
    scoreRealTimeOfFlightMap.put(1.488, 0.0);

    scoreRealFlywheelSpeedMap.put(2.462, 1150.0);
    scoreRealHoodAngleMap.put(2.462, 19.0);
    // scoreRealTimeOfFlightMap.put(, );

    scoreRealFlywheelSpeedMap.put(3.62, 1260.0);
    scoreRealHoodAngleMap.put(3.62, 26.5);

    scoreRealFlywheelSpeedMap.put(4.513, 1300.0);
    scoreRealHoodAngleMap.put(4.513, 28.0);

    scoreRealFlywheelSpeedMap.put(5.419, 1410.0);
    scoreRealHoodAngleMap.put(5.419, 32.5);

    // scoreRealFlywheelSpeedMap.put(, );
    // scoreRealHoodAngleMap.put(, );

    if (Constants.currentMode == Constants.Mode.SIM) {
      addScoreSimMeasurements();
    }
  }

  public record LaunchingParameters(
      boolean isValid, double turretAngle, double hoodAngle, double flywheelSpeed) {}

  public double feedingHoodAngle(double distToWall) {
    return feedRealHoodAngleMap.get(distToWall);
  }

  public double feedingFlywheelSpeed(double distToWall) {
    return feedRealFlywheelSpeedMap.get(distToWall);
  }

  public LaunchingParameters getLaunchingParameters(
      boolean isScoring, boolean isReal, boolean lookahead) {
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
    // if (isScoring) { // TODO change if feeding and not scoring
    targetPose = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    // } else {
    //   if (turretPosition.getY() > FieldConstants.fieldWidth / 2) {
    //     targetPose =
    //         AllianceFlipUtil.apply(
    //             new Translation2d(AllianceFlipUtil.applyX(0), FieldConstants.fieldWidth - 0.5));
    //   } else {
    //     targetPose = AllianceFlipUtil.apply(new Translation2d(AllianceFlipUtil.applyX(0), 0.5));
    //   }
    // }
    Logger.recordOutput("Shooter/Turret/Target", new Pose2d(targetPose, new Rotation2d()));

    Logger.recordOutput("Shooter/Turret/CurrentTurretPose", turretPosition);
    double turretToTargetDistance = targetPose.getDistance(turretPosition.getTranslation()) + 0.3;

    // Twist2d fieldRelativeRobotVelocity = poseManager.getFieldVelocity();
    // double robotAngle = robotPose.getRotation().getRadians();
    // double turretVelocityX =
    //     fieldRelativeRobotVelocity.dx
    //         + fieldRelativeRobotVelocity.dtheta
    //             * (turretCenter.getY() * Math.cos(robotAngle)
    //                 - turretCenter.getX() * Math.sin(robotAngle));
    // double turretVelocityY =
    //     fieldRelativeRobotVelocity.dy
    //         + fieldRelativeRobotVelocity.dtheta
    //             * (turretCenter.getX() * Math.cos(robotAngle)
    //                 - turretCenter.getY() * Math.sin(robotAngle));

    // double timeOfFlight;
    // Pose2d lookeaheadPose = turretPosition;
    // double lookaheadTurretToTargetDistance = turretToTargetDistance;
    // for (int i = 0; i < 10; i++) {
    //   //   timeOfFlight = 0; // change this back later
    //   // timeOfFlight = 0.5; // TODO: replace with actual time of flight calculation
    //   timeOfFlight = lookahead ? timeOfFlightMap.get(lookaheadTurretToTargetDistance) : 0;
    //   double offsetX = turretVelocityX * timeOfFlight;
    //   double offsetY = turretVelocityY * timeOfFlight;
    //   lookeaheadPose =
    //       new Pose2d(
    //           turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
    //           turretPosition.getRotation());
    //   lookaheadTurretToTargetDistance = targetPose.getDistance(lookeaheadPose.getTranslation());
    // }

    // Logger.recordOutput(
    //     "Shooter/Turret/LookaheadTurretToTargetDist", lookaheadTurretToTargetDistance);

    // Logger.recordOutput("Shooter/Turret/LookaheadTurretPose", lookeaheadPose);

    // lookaheadTurretToTargetDistance += isReal ? 0.33 : 0; // shoot to back of hub, not center

    // turretAngle =
    //     targetPose.minus(lookeaheadPose.getTranslation()).getAngle().getDegrees()
    //         - poseManager.getRotation().getDegrees();

    LaunchingParameters params =
        new LaunchingParameters(
            // minDist < lookaheadTurretToTargetDistance && lookaheadTurretToTargetDistance <
            // maxDist,
            true,
            turretAngle, // doesnt matter rn
            hoodAngleMap.get(
                turretToTargetDistance), // mybe add arbitrary value if accuracy bad to shoot back
            // of hub
            flywheelSpeedMap.get(turretToTargetDistance));
    return params;
  }

  public LaunchingParameters getSOTMParameters(boolean isScoring, boolean isReal) {
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
    double turretToTargetDistance = targetPose.getDistance(turretPosition.getTranslation()) + 0.3;

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
      //   timeOfFlight = 0; // change this back later
      // timeOfFlight = 0.5; // TODO: replace with actual time of flight calculation
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;
      lookeaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = targetPose.getDistance(lookeaheadPose.getTranslation());
    }

    Logger.recordOutput(
        "Shooter/Turret/LookaheadTurretToTargetDist", lookaheadTurretToTargetDistance);

    Logger.recordOutput("Shooter/Turret/LookaheadTurretPose", lookeaheadPose);

    lookaheadTurretToTargetDistance += 0.2; // shoot to back of hub, not center

    turretAngle =
        targetPose.minus(lookeaheadPose.getTranslation()).getAngle().getDegrees()
            - poseManager.getRotation().getDegrees();

    LaunchingParameters params =
        new LaunchingParameters(
            // minDist < lookaheadTurretToTargetDistance && lookaheadTurretToTargetDistance <
            // maxDist,
            true,
            turretAngle, // doesnt matter rn
            hoodAngleMap.get(
                lookaheadTurretToTargetDistance), // mybe add arbitrary value if accuracy bad to
            // shoot back
            // of hub
            flywheelSpeedMap.get(lookaheadTurretToTargetDistance));
    return params;
  }

  void addScoreSimMeasurements() {
    scoreSimFlywheelSpeedMap.put(1.256, 1200.0);
    scoreSimHoodAngleMap.put(1.256, 14.6);

    scoreSimFlywheelSpeedMap.put(1.454, 1200.0);
    scoreSimHoodAngleMap.put(1.454, 18.0);

    scoreSimFlywheelSpeedMap.put(2.117, 1300.0);
    scoreSimHoodAngleMap.put(2.117, 17.5);

    scoreSimFlywheelSpeedMap.put(2.302, 1300.0);
    scoreSimHoodAngleMap.put(2.302, 24.0);

    scoreSimFlywheelSpeedMap.put(2.569, 1400.0);
    scoreSimHoodAngleMap.put(2.569, 26.0);

    scoreSimFlywheelSpeedMap.put(2.700, 1350.0);
    scoreSimHoodAngleMap.put(2.700, 26.0);

    scoreSimFlywheelSpeedMap.put(3.074, 1500.0);
    scoreSimHoodAngleMap.put(3.074, 20.5);

    scoreSimFlywheelSpeedMap.put(3.203, 1350.0);
    scoreSimHoodAngleMap.put(3.203, 33.0);

    scoreSimFlywheelSpeedMap.put(3.399, 1400.0);
    scoreSimHoodAngleMap.put(3.399, 35.0);

    scoreSimFlywheelSpeedMap.put(3.589, 1450.0);
    scoreSimHoodAngleMap.put(3.589, 35.0);

    scoreSimFlywheelSpeedMap.put(3.797, 1450.0);
    scoreSimHoodAngleMap.put(3.797, 35.0);

    scoreSimFlywheelSpeedMap.put(4.025, 1500.0);
    scoreSimHoodAngleMap.put(4.025, 40.0);

    scoreSimFlywheelSpeedMap.put(4.212, 1500.0);
    scoreSimHoodAngleMap.put(4.212, 35.0);

    scoreSimFlywheelSpeedMap.put(4.458, 1500.0);
    scoreSimHoodAngleMap.put(4.458, 33.0);

    scoreSimFlywheelSpeedMap.put(4.607, 1600.0);
    scoreSimHoodAngleMap.put(4.607, 38.0);

    scoreSimFlywheelSpeedMap.put(4.807, 1600.0);
    scoreSimHoodAngleMap.put(4.807, 37.0);

    scoreSimFlywheelSpeedMap.put(5.117, 1600.0);
    scoreSimHoodAngleMap.put(5.117, 35.0);

    scoreSimFlywheelSpeedMap.put(5.312, 1650.0);
    scoreSimHoodAngleMap.put(5.312, 38.0);
    scoreSimTimeOfFlightMap.put(4.044688, 0.857525);
    scoreSimTimeOfFlightMap.put(3.503173, 0.89692);
    scoreSimTimeOfFlightMap.put(2.976572, 1.140367);
    scoreSimTimeOfFlightMap.put(2.406895, 0.979472);
    scoreSimTimeOfFlightMap.put(1.266091, 0.899978);
    scoreSimTimeOfFlightMap.put(1.806139, 0.940424);
    scoreSimTimeOfFlightMap.put(4.524281, 1.019291);
    scoreSimTimeOfFlightMap.put(5.048564, 1.081496);
    scoreSimTimeOfFlightMap.put(5.55331, 1.078915);
    // scoreSimTimeOfFlightMap.put(, );
    // scoreSimTimeOfFlightMap.put(, );

  }
}
