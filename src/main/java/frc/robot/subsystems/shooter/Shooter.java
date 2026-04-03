package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.*;
import static frc.robot.FieldConstants.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.subsystems.shooter.ShooterUtil.*;
import static frc.robot.util.FuelSim.Hub.*;
import static frc.robot.util.GeomUtil.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.LeftTrench;
import frc.robot.FieldConstants.LinesHorizontal;
import frc.robot.FieldConstants.LinesVertical;
import frc.robot.FieldConstants.RightTrench;
import frc.robot.subsystems.leds.Leds;
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
import frc.robot.util.ShiftHelpers;
import frc.robot.util.ShiftHelpers.Shift;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

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

  private final LoggedTunableNumber feedingVelocity =
      new LoggedTunableNumber("Shooter/feedingVelocity", 1000);
  private final LoggedTunableNumber farFeedingVelocity =
      new LoggedTunableNumber("Shooter/FarFeedingVelocity", 2000);
  private final LoggedTunableNumber feedingHoodAngle =
      new LoggedTunableNumber("Shooter/feedingHoodAngle", 45);
  private final LoggedTunableNumber farFeedingHoodAngle =
      new LoggedTunableNumber("Shooter/farFeedingHoodAngle", 42);

  private final LoggedNetworkBoolean eFixedAngle = new LoggedNetworkBoolean("fixed angle", false);

  private final LoggedTunableNumber clearBalls = new LoggedTunableNumber("Shooter/ClearBalls", 0);

  private final ShooterUtil shooterUtil;

  private final PoseManager poseManager;

  private boolean isShooting = false;
  private boolean isScoring = true;
  private boolean hoodIsSafe = true;
  private boolean isAutoFeedVsScore = true;
  private boolean isAutoFeedVsScorePermaDisable = false;
  private boolean isClose = true;
  private boolean lastInAZ = true;

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

  private Debouncer trenchAvoidenceDebouncer =
      new Debouncer(trenchAvoidenceDebouncerTime.get(), DebounceType.kRising);

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

    if (DriverStation.isDisabled()
        && ShiftHelpers.getNextShift().orElse(Shift.AUTO) == Shift.TRANSITION) {
      isAutoFeedVsScore = true;
    }

    if (DriverStation.isDisabled()) isShooting = false;

    boolean inAZ =
        AllianceFlipUtil.applyX(poseManager.getPose().getX())
            < FieldConstants.LinesVertical.allianceZone + 0.75;
    if (isAutoFeedVsScore) {
      isScoring = inAZ;
      lastInAZ = inAZ;
    } else if (lastInAZ != inAZ) {
      isAutoFeedVsScore = true;
    }

    if (isAutoFeedVsScorePermaDisable) {
      isAutoFeedVsScore = false;
    }

    Logger.recordOutput("Shooter/isScoring", isScoring);
    Logger.recordOutput("Shooter/isShooting", isShooting);
    Logger.recordOutput("Shooter/isClose", isClose);
    Logger.recordOutput("Shooter/Hood/isSafe", hoodIsSafe);

    turret.setIsShooting(isShooting);
    flywheels.setIsShooting(isShooting);

    myX = poseManager.getPose().getX() - LinesVertical.center;
    myY = poseManager.getPose().getY() - LinesHorizontal.center;

    // rlly fucking cooked way to do this but dont comment pls
    // hmmmmmmm
    // comment
    if (clearBalls.get() == 1) {
      fuelSim.clearFuel();
    }

    // * Real Mode
    // hood.setAngle(fakeHoodAngle.get());
    // flywheels.setVelocity(fakeFlywheelVelocity.get());
    // // turret.setTarget(fakeTurretAngle.get(), fakeTurretVelocity.get());
    Pose2d robotPose = poseManager.getPose();
    Translation2d targetPose = // TODO change if feeding and not scoring
        // AllianceFlipUtil.apply(new Translation2d(AllianceFlipUtil.applyX(0), 0.5));
        AllianceFlipUtil.apply(
            FieldConstants.Hub.topCenterPoint.toTranslation2d().plus(new Translation2d(0, 0)));
    Logger.recordOutput("Shooter/Turret/turretTarget", new Pose2d(targetPose, new Rotation2d()));

    // Set turret angle
    Pose2d turretPosition =
        robotPose.transformBy(
            new Transform2d(
                turretCenter.getTranslation().toTranslation2d(), poseManager.getRotation()));
    Logger.recordOutput("Shooter/Turret/turretpose", turretPosition);
    double turretAngle =
        targetPose.minus(turretPosition.getTranslation()).getAngle().getDegrees()
            - poseManager.getRotation().getDegrees();
    turret.setTarget(turretAngle);

    Logger.recordOutput(
        "Shooter/Turret/DistToTarget", turretPosition.getTranslation().getDistance(targetPose));

    // Set hood and flywheel
    // if (DriverStation.isAutonomous()) {
    //   hood.setAngle(autoHoodAngle.get());
    //   flywheels.setVelocity(autoFlywheelVelocity.get());
    // } else {
    LaunchingParameters solution =
        shooterUtil.getLaunchingParameters(true, true, false);
    if (solution.isValid()) {
      // double turretAngle = solution.turretAngle() - poseManager.getRotation().getDegrees();
      // turret.setTarget(turretAngle);

      // TODO uncomment only the below two lines when ready to use interp map
      hood.setAngle(solution.hoodAngle());
      flywheels.setVelocity(solution.flywheelSpeed());
    }

    // For feeding
    double distToWall = AllianceFlipUtil.applyX(turretPosition.getX());
    Logger.recordOutput("Shooter/Turret/distToWall", distToWall);

    if (!isScoring) {
      turret.setTarget(180 - AllianceFlipUtil.apply(poseManager.getRotation()).getDegrees());
      // turret.setTarget(fakeTurretAngle.get());
      hood.setAngle(shooterUtil.feedingHoodAngle(distToWall));
      // hood.setAngle(fakeHoodAngle.get());

      flywheels.setVelocity(shooterUtil.feedingFlywheelSpeed(distToWall));

      // To shoot towards a certain location if turret starts working
      if (myY > 0f) {
        // To do add shoot to near the middle top
      } else {
        // To do add shoot to near the middle bottom
      }
    }
    // } // see above part about autos if you're using this curly brace

    // For testing / tuning
    // flywheels.setVelocity(fakeFlywheelVelocity.get());
    // hood.setAngle(fakeHoodAngle.get());
    // turret.setTarget(fakeTurretAngle.get());

    // Hood safety
    if (!isShooting || !hoodIsSafe) {
      hood.setAngle(HoodConstants.minPositionDegs);
    }

    if (eFixedAngle.getAsBoolean()) {
      turret.setTarget(turretFixedAngle);
    }

    // For fuel sim
    if (Constants.currentMode == Constants.Mode.SIM
        && isShooting
        && fuelLaunchTimer.hasElapsed(0.5)) {
      //   fuelSim.launchFuel(
      //       MetersPerSecond.of(fakeFlywheelVelocity.get() * 2 * Math.PI * WheelRadius / 60),
      //       Degrees.of(90 - fakeHoodAngle.get()),
      //       Degrees.of(fakeTurretAngle.get()),
      //       turretCenter);
      fuelLaunchTimer.restart();
      fuelDelayTimer.restart();
      shoot = true;
      params = shooterUtil.getLaunchingParameters(true, false, true);
    }
    if (fuelDelayTimer.hasElapsed(fuelDelay.get())
        && shoot
        && Constants.currentMode == Constants.Mode.SIM) {
      shoot = false;
      fuelSim.launchFuel(
          MetersPerSecond.of(params.flywheelSpeed() * 2 * Math.PI * WheelRadius / 60),
          Degrees.of(90 - params.hoodAngle()),
          Degrees.of(params.turretAngle()),
          turretCenter);
    }
    if (prevHubScore != BLUE_HUB.getScore() + RED_HUB.getScore()
        && Constants.currentMode == Constants.Mode.SIM) {
      Logger.recordOutput("Shooter/FlightTime", fuelLaunchTimer.get());
      prevHubScore = BLUE_HUB.getScore() + RED_HUB.getScore();
    }

    Leds.getInstance().isShooting = getShooting();
    Leds.getInstance().isScoring = getScoring();
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

  public boolean getScoring() {
    return isScoring;
  }

  public Command setHoodIsSafe(boolean bool) {
    return runOnce(() -> hoodIsSafe = bool).withName("SetHoodIsSafe");
  }

  public Command overrideSetScoring(boolean scoring) {
    return runOnce(
            () -> {
              isScoring = scoring;
              isAutoFeedVsScore = false;
            })
        .withName("OverrideSetScoring" + scoring);
  }

  public Command permaDisableAutoFeedvsScore() {
    return runOnce(() -> isAutoFeedVsScorePermaDisable = true);
  }

  // TODO Sean needs to make this work better. Make it go from minimum safe angle to maximum safe
  // angle.
  public Command testTurret() {
    return run(() -> turret.setTarget(90), turret).withName("TestTurret");
  }

  public Command testHood() {
    return run(() -> hood.setAngle(HoodConstants.maxPositionDegs), hood)
        .until(hood::atGoal)
        .andThen(() -> hood.setAngle(HoodConstants.minPositionDegs), hood)
        .until(hood::atGoal)
        .withName("TestHood");
  }

  public Command testFlywheelsRPM() {
    return run(() -> flywheels.setVelocity(700), flywheels).withName("TestFlywheels");
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

    if (trenchAvoidenceDebouncer.calculate(inXRange & inYRange)) {
      if (trenchAvoidenceDebouncer.calculate(
          trenchUpAndMovingUp || trenchDownAndMovingDown || upwardMovmentCorospondWithDirection)) {
        hood.setAngle(0);
      }
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
