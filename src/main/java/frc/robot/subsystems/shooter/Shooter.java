package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.currentMode;
import static frc.robot.FieldConstants.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.subsystems.shooter.ShooterUtil.*;
import static frc.robot.util.FuelSim.Hub.*;
import static frc.robot.util.GeomUtil.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.LeftTrench;
import frc.robot.FieldConstants.LinesHorizontal;
import frc.robot.FieldConstants.LinesVertical;
import frc.robot.FieldConstants.RightTrench;
import frc.robot.subsystems.shooter.ShooterUtil.LaunchingParameters;
import frc.robot.subsystems.shooter.flywheels.Flywheels;
import frc.robot.subsystems.shooter.flywheels.Flywheels.FlywheelsState;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FuelSim;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PoseManager;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class Shooter extends VirtualSubsystem {
  private final Flywheels flywheels;
  private final Turret turret;
  private final Hood hood;

  private final ShooterVisualizer measuredVisualizer =
      new ShooterVisualizer("Measured", Color.kRed);
  private final ShooterVisualizer setpointVisualizer =
      new ShooterVisualizer("Setpoint", Color.kBlue);
  private final LoggedTunableNumber fakeTurretAngle =
      new LoggedTunableNumber("Shooter/FakeTurretAngle", 0);
  // private double fakeTurretAngle = 0;
  private final LoggedTunableNumber fakeTurretVelocity =
      new LoggedTunableNumber("Shooter/FakeTurretVelocity", 0);
  private final LoggedTunableNumber fakeHoodAngle =
      new LoggedTunableNumber("Shooter/FakeHoodAngle", 0);
  private final LoggedTunableNumber fakeFlywheelVelocity =
      new LoggedTunableNumber("Shooter/FakeFlywheelVelocity", 0);
  private final LoggedTunableNumber farFlywheelVelocity =
      new LoggedTunableNumber("Shooter/FarVelocity", 1600);
  private final LoggedTunableNumber hubFlywheelVelocity =
      new LoggedTunableNumber("Shooter/HubVelocity", 1100);

  private final LoggedTunableNumber fakeFeedingVelocity =
      new LoggedTunableNumber("Shooter/FeedingVelocity", 1600);
  private final LoggedTunableNumber fakeFeedingHoodAngle =
      new LoggedTunableNumber("Shooter/fakeFeedingHoodAngle", 30);

  private final LoggedTunableNumber clearBalls = new LoggedTunableNumber("Shooter/ClearBalls", 0);

  private final ShooterUtil shooterUtil;

  private final PoseManager poseManager;

  private boolean isShooting = false;
  private boolean isScoring = true;
  private boolean hoodIsSafe = false;
  private boolean isAutoFeedVsScore = true;
  private boolean isClose = true;

  private double myX;
  private double myY;
  private double dX;
  private double dY;

  private double closeBorder;
  private double farBorder;
  private double leftBorder;

  private boolean inXRange;
  private boolean inYRange;

  private boolean approachingFromAllianceSide;
  private boolean approachingFromRight;

  private boolean trenchUpAndMovingUp;
  private boolean trenchDownAndMovingDown;

  private boolean onRightAndMovingRight;
  private boolean onLeftAndMovingLeft;

  private boolean upwardMovmentCorospondWithDirection;
  private FuelSim fuelSim;
  private final Timer fuelLaunchTimer = new Timer();
  private final Timer fuelDelayTimer = new Timer();
  private final LoggedTunableNumber fuelDelay = new LoggedTunableNumber("Shooter/FuelDelay", 0.02);
  private boolean shoot = false;
  LaunchingParameters params;

  private int prevHubScore = 0;

  public Shooter(
      Flywheels flywheels, Turret turret, Hood hood, PoseManager poseManager, FuelSim fuelSim) {
    this.flywheels = flywheels;
    this.turret = turret;
    this.hood = hood;
    this.poseManager = poseManager;
    this.fuelSim = fuelSim;
    this.shooterUtil = new ShooterUtil(this.poseManager);

    fuelLaunchTimer.start();
  }

  public void periodic() {
    // if (DriverStation.isAutonomous()) {
    //   isAutoFeedVsScore = true;
    // }

    if (isAutoFeedVsScore) {
      isScoring =
          AllianceFlipUtil.applyX(poseManager.getPose().getX())
              < FieldConstants.LinesVertical.allianceZone;
    }

    Logger.recordOutput("Shooter/isScoring", isScoring);
    Logger.recordOutput("Shooter/isShooting", isShooting);
    Logger.recordOutput("Shooter/isClose", isClose);
    Logger.recordOutput("Shooter/Hood/isSafe", hoodIsSafe);

    // turret.setIsShooting(isShooting);
    turret.setIsShooting(true);
    flywheels.setIsShooting(isShooting);

    myX = poseManager.getPose().getX() - LinesVertical.center;
    myY = poseManager.getPose().getY() - LinesHorizontal.center;

    // LaunchingParameters solution =
    //     shooterUtil.getLaunchingParameters(isScoring, Constants.currentMode !=
    // Constants.simMode);
    // if (solution.isValid()) {
    //   double turretAngle = solution.turretAngle() - poseManager.getRotation().getDegrees();
    //   double turretVelocity =
    //       solution.turretVelocity() -
    // Units.radiansToDegrees(poseManager.getRobotVelocity().dtheta);
    //   turret.setTarget(turretAngle, turretVelocity);
    //   hood.setAngle(solution.hoodAngle());
    //   flywheels.setVelocity(solution.flywheelSpeed());
    //   KickerConstants.RPMSetpoint = solution.kickerSpeed();
    // }

    // rlly fucking cooked way to do this but dont comment pls
    // hmmmmmmm
    // comment
    if (clearBalls.get() == 1) {
      fuelSim.clearFuel();
    }

    if (currentMode != Constants.simMode) {
      // * Real Mode
      // if (isScoring) {
      //   hood.setAngle(fakeHoodAngle.get());
      //   flywheels.setVelocity(fakeFlywheelVelocity.get());
      // } else {
      //   flywheels.setVelocity(fakeFeedingVelocity.get());
      //   hood.setAngle(fakeFeedingHoodAngle.get());
      // }
      hood.setAngle(fakeHoodAngle.get());
      flywheels.setVelocity(fakeFlywheelVelocity.get());
      turret.setTarget(fakeTurretAngle.get(), fakeTurretVelocity.get());
      Pose2d robotPose = poseManager.getPose();
      Translation2d targetPose = // TODO change if feeding and not scoring
          AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
      Pose2d turretPosition =
          robotPose.transformBy(
              new Transform2d(
                  turretCenter.getTranslation().toTranslation2d(),
                  turretCenter.getRotation().toRotation2d()));
      // double turretAngle =
      //     targetPose.minus(turretPosition.getTranslation()).getAngle().getDegrees() + 180;
      // turret.setTarget(turretAngle, 0);

      // hood.setAngle(hoodIsSafe ? fakeHoodAngle.get() : HoodConstants.minPositionDegs);
      // if (isScoring) {
      //   if (isClose) {
      //     flywheels.setVelocity(hubFlywheelVelocity.get());
      //   } else {
      //     flywheels.setVelocity(farFlywheelVelocity.get());
      //   }
      // } else {
      //   flywheels.setVelocity(fakeFeedingVelocity.get());
      // }
    } else {
      turret.setTarget(fakeTurretAngle.get(), fakeTurretVelocity.get());
      hood.setAngle(fakeHoodAngle.get());
      flywheels.setVelocity(fakeFlywheelVelocity.get());
    }
    // Avi commented this out because real robot is not ready for it
    // TrenchAvoidence();

    measuredVisualizer.update(turret.getPositionDegs(), hood.getAngleDeg());
    setpointVisualizer.update(0, fakeTurretVelocity.get());
    // measuredVisualizer.update(fakeTurretAngle.get(), fakeHoodAngle.get());

    if (myY > 0f) {
      // To do add shoot to near the middle top
    } else {
      // To do add shoot to near the middle bottom
    }

    if (Constants.currentMode == Constants.simMode
        && isShooting
        && fuelLaunchTimer.hasElapsed(0.5)) {
      // fuelSim.launchFuel(
      //     MetersPerSecond.of(fakeFlywheelVelocity.get() * 2 * Math.PI * WheelRadius / 60),
      //     Degrees.of(90 - fakeHoodAngle.get()),
      //     Degrees.of(fakeTurretAngle.get()),
      //     turretCenter);
      fuelLaunchTimer.restart();
      fuelDelayTimer.restart();
      shoot = true;
      params = shooterUtil.getLaunchingParameters(true, false);
    }
    if (fuelDelayTimer.hasElapsed(fuelDelay.get()) && shoot) {
      shoot = false;
      fuelSim.launchFuel(
          MetersPerSecond.of(params.flywheelSpeed() * 2 * Math.PI * WheelRadius / 60),
          Degrees.of(90 - params.hoodAngle()),
          Degrees.of(params.turretAngle()),
          turretCenter);
    }
    if (prevHubScore != BLUE_HUB.getScore() + RED_HUB.getScore()) {
      Logger.recordOutput("Shooter/FlightTime", fuelLaunchTimer.get());
      prevHubScore = BLUE_HUB.getScore() + RED_HUB.getScore();
    }
    Logger.recordOutput(
        "Shooter/Turret/DistToHub",
        poseManager
            .getPose()
            .plus(toTransform2d(turretCenter.getX(), turretCenter.getY()))
            .getTranslation()
            .getDistance(
                AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())));
  }

  public boolean readyToShoot() {
    return turret.atGoal() && hood.atGoal() && flywheels.atGoal();
  }

  public Command setShooting(boolean shooting) {
    return runOnce(() -> isShooting = shooting, flywheels, turret, hood)
        .withName(shooting ? "StartShooting" : "StopShooting");
  }

  public Command toggleShooting() {
    return runOnce(() -> isShooting = !isShooting, flywheels, turret, hood)
        .withName("LowLevelToggleShooting");
  }

  public boolean getShooting() {
    return isShooting;
  }

  public Command toggleHoodIsSafe() {
    return runOnce(() -> hoodIsSafe = !hoodIsSafe, hood).withName("ToggleHoodIsSafe");
  }

  public Command overrideSetScoring(boolean scoring) {
    return runOnce(
            () -> {
              isScoring = scoring;
              isAutoFeedVsScore = false;
            })
        .withName("OverrideSetScoring" + scoring);
  }

  // TODO Sean needs to make this work better. Make it go from minimum safe angle to maximum safe
  // angle.
  public Command testTurret() {
    return run(() -> turret.setTarget(90, 0), turret).withName("TestTurret");
  }

  public Command testHood() {
    return run(() -> hood.setAngle(HoodConstants.maxPositionDegs), hood)
        .until(hood::atGoal)
        .andThen(() -> hood.setAngle(HoodConstants.minPositionDegs), hood)
        .until(hood::atGoal)
        .withName("TestHood");
  }

  public Command testFlywheelsRPM() {
    return run(() -> flywheels.setVelocity(5000), flywheels)
        .withTimeout(1)
        .withName("TestFlywheels");
  }

  public Command setIsClose(boolean isClose) {
    return runOnce(() -> this.isClose = isClose);
  }

  // public void incrementFlywheelVelocity(double increment) {
  //   flywheelVelocity += increment;
  //   if(flywheelVelocity < 500) {
  //     flywheelVelocity = 500;
  //   } else if(flywheelVelocity > 5000) {
  //     flywheelVelocity = 5000;
  //   }
  // }

  // public Command increaseFlywheelVelocity() {
  //   return runOnce(() -> incrementFlywheelVelocity(10), flywheels)
  //       .withTimeout(0.1)
  //       .withName("IncreaseFlywheelVelocity");
  // }

  // public Command decreaseFlywheelVelocity() {
  //   return runOnce(() -> incrementFlywheelVelocity(-10), flywheels)
  //       .withTimeout(0.1)
  //       .withName("DecreaseFlywheelVelocity");
  // }

  public Command testFlywheelsVolts() {
    return runOnce(() -> flywheels.setState(FlywheelsState.VOLTS)).withName("TestFlywheelsVolts");
  }

  private void TrenchAvoidence() {
    SetupTrenchAvoidenceInputs();
    DropHood();
    LogTrenchAvoidence();
  }

  private void SetupTrenchAvoidenceInputs() {

    dX = poseManager.getFieldVelocity().dx;
    dY = poseManager.getFieldVelocity().dy;

    closeBorder = RightTrench.openingTopRight.getX() - trenchRadius;
    farBorder = RightTrench.openingTopRight.getX() + trenchRadius;
    leftBorder = LeftTrench.openingTopRight.getY() - trenchRadius;

    // transpose so that zero zero is the center

    closeBorder -= LinesVertical.center;
    farBorder -= LinesVertical.center;
    leftBorder -= LinesHorizontal.center;

    inXRange = Math.abs(myX) < Math.abs(closeBorder) && Math.abs(myX) > Math.abs(farBorder);
    inYRange = Math.abs(myY) > leftBorder;

    approachingFromAllianceSide = dX > 0f;
    approachingFromRight = dY > 0f;

    trenchUpAndMovingUp =
        Math.abs(myX) < Math.abs(RightTrench.openingTopRight.getX() - LinesVertical.center)
            && approachingFromAllianceSide;
    trenchDownAndMovingDown =
        Math.abs(myX) > Math.abs(RightTrench.openingTopRight.getX() - LinesVertical.center)
            && !approachingFromAllianceSide;

    upwardMovmentCorospondWithDirection =
        Math.abs(myY) > LeftTrench.openingTopRight.getY()
            & ((Math.abs(myY) == myY) & approachingFromRight);
  }

  private void DropHood() {

    if (inXRange & inYRange) {
      if (trenchUpAndMovingUp || trenchDownAndMovingDown || upwardMovmentCorospondWithDirection) {
        hood.setAngle(0);
      } else {
        // swap for solution.hoodAngle() when working
        hood.setAngle(341.5);
      }
    } else {
      // swap for solution.hoodAngle() when working
      hood.setAngle(341.5);
    }
  }

  // public Command incrementTurretAngle() {
  //   return run(() -> fakeTurretAngle++);
  // }

  // public Command decrementTurretAngle() {
  //   return run(() -> fakeTurretAngle--);
  // }

  private void LogTrenchAvoidence() {
    Logger.recordOutput("Controls/Trench Avoidence/closeBorder", closeBorder);
    Logger.recordOutput("Controls/Trench Avoidence/farBorder", farBorder);
    Logger.recordOutput("Controls/Trench Avoidence/leftBorder", leftBorder);

    Logger.recordOutput("Controls/Trench Avoidence/inXRange", inXRange);
    Logger.recordOutput("Controls/Trench Avoidence/inYRange", inYRange);

    Logger.recordOutput("Controls/Trench Avoidence/trenchUpAndMovingUp", trenchUpAndMovingUp);
    Logger.recordOutput(
        "Controls/Trench Avoidence/trenchDownAndMovingDown", trenchDownAndMovingDown);
    Logger.recordOutput(
        "Controls/Trench Avoidence/upwardMovmentCorospondWithDirection",
        upwardMovmentCorospondWithDirection);

    Logger.recordOutput(
        "Controls/Trench Avoidence/approachingFromAllianceSide", approachingFromAllianceSide);
    Logger.recordOutput("Controls/Trench Avoidence/approachingFromRight", approachingFromRight);

    Logger.recordOutput("Controls/Trench Avoidence/myX", myX);
    Logger.recordOutput("Controls/Trench Avoidence/myY", myY);

    Logger.recordOutput("Controls/Trench Avoidence/dX", dX);
    Logger.recordOutput("Controls/Trench Avoidence/dY", dY);
  }
}
