package frc.robot.subsystems.shooter;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.FieldConstants.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.subsystems.shooter.ShooterUtil.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.LeftTrench;
import frc.robot.FieldConstants.RightTrench;
import frc.robot.subsystems.shooter.ShooterUtil.*;
import frc.robot.subsystems.shooter.flywheels.Flywheels;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
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
  private final LoggedTunableNumber fakeTurretVelocity =
      new LoggedTunableNumber("Shooter/FakeTurretVelocity", 0);
  private final LoggedTunableNumber fakeHoodAngle =
      new LoggedTunableNumber("Shooter/FakeHoodAngle", 0);
  private final LoggedTunableNumber fakeFlywheelVelocity =
      new LoggedTunableNumber("Shooter/FakeFlywheelVelocity", 0);

  private final ShooterUtil shooterUtil;

  private final PoseManager poseManager;

  private boolean isShooting = false;
  private boolean isScoring = false;

  private double myX;
  private double myY;
  private double dX;
  private double dY;

  private double closeBorder;
  private double farBorder;
  private double rightBorder;
  private double leftBorder;

  private boolean inXRange;
  private boolean inYRange;

  private boolean approachingFromAllianceSide;
  private boolean approachingFromRight;

  private boolean trenchUpAndMovingUp;
  private boolean trenchDownAndMovingDown;

  private boolean onRightAndMovingRight;
  private boolean onLeftAndMovingLeft;
  private FuelSim fuelSim;

  public Shooter(
      Flywheels flywheels, Turret turret, Hood hood, PoseManager poseManager, FuelSim fuelSim) {
    this.flywheels = flywheels;
    this.turret = turret;
    this.hood = hood;
    this.poseManager = poseManager;
    this.fuelSim = fuelSim;
    this.shooterUtil = new ShooterUtil(this.poseManager);

    // TODO add default commands
  }

  public void periodic() {
    Pose3d goalPose = new Pose3d();

    isScoring = poseManager.getPose().getX() < FieldConstants.LinesVertical.allianceZone;
    Logger.recordOutput("Subsystems/Shooter/isScoring", isScoring);
    turret.setIsShooting(isShooting);
    flywheels.setIsShooting(isShooting);

    // LaunchingParameters solution = shooterUtil.getScoringParameters();
    // if (solution.isValid()) {
    //   double turretAngle = solution.turretAngle() - poseManager.getRotation().getDegrees();
    //   double turretVelocity =
    //       solution.turretVelocity() -
    // Units.radiansToDegrees(poseManager.getRobotVelocity().dtheta);
    //   turret.setTarget(turretAngle, turretVelocity);
    //   hood.setAngle(solution.hoodAngle());
    //   flywheels.setVelocity(solution.flywheelSpeed());
    //   if(Constants.currentMode == Constants.simMode) {
    //
    // fuelSim.launchFuel(MetersPerSecond.of(Units.degreesToRadians(solution.flywheelSpeed())*WheelRadius), new Rotation2d(solution.hoodAngle()).getMeasure(), new Rotation2d(turretAngle).getMeasure(), turretCenter.getMeasureZ());
    //   }
    // }

    turret.setTarget(fakeTurretAngle.get(), fakeTurretVelocity.get());
    hood.setAngle(fakeHoodAngle.get());
    flywheels.setVelocity(fakeFlywheelVelocity.get());

    TrenchAvoidence();

    // TODO uncomment when ready to test
    measuredVisualizer.update(turret.getPositionDegs(), hood.getAngle());
    setpointVisualizer.update(0, fakeTurretVelocity.get());
    // measuredVisualizer.update(fakeTurretAngle.get(), fakeHoodAngle.get());
  }

  public boolean readyToShoot() {
    return turret.atGoal() && hood.atGoal() && flywheels.atGoal();
  }

  public Command setShooting(boolean shooting) {
    return runOnce(() -> isShooting = shooting)
        .alongWith(runOnce(() -> turret.setIsShooting(shooting)))
        .alongWith(runOnce(() -> flywheels.setIsShooting(shooting)));
  }

  public Command setScoring(boolean scoring) {
    return runOnce(() -> isScoring = scoring);
  }

  private void TrenchAvoidence() {
    SetupTrenchAvoidenceInputs();
    DropHood();
    LogTrenchAvoidence();
  }

  private void SetupTrenchAvoidenceInputs() {

    myX = poseManager.getPose().getX();
    myY = poseManager.getPose().getY();
    dX = poseManager.getFieldVelocity().dx;
    dY = poseManager.getFieldVelocity().dy;

    closeBorder = RightTrench.openingTopRight.getX() - trenchRadius;
    farBorder = RightTrench.openingTopRight.getX() + trenchRadius;
    rightBorder = RightTrench.openingTopLeft.getY() + trenchRadius;
    leftBorder = LeftTrench.openingTopRight.getY() - trenchRadius;

    // transpose so that zero zero is the center
    myX -= LinesVertical.center;
    myY -= LinesHorizontal.center;

    closeBorder -= LinesVertical.center;
    farBorder -= LinesVertical.center;
    rightBorder -= LinesHorizontal.center;
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

    onRightAndMovingRight =
        Math.abs(myY) < RightTrench.openingTopLeft.getY() - LinesHorizontal.center
            && approachingFromRight;
    onLeftAndMovingLeft =
        myY > LeftTrench.openingTopLeft.getY() - LinesHorizontal.center && !approachingFromRight;
  }

  private void DropHood() {

    if (inXRange && inYRange) {

      if (onRightAndMovingRight || onLeftAndMovingLeft) {
        hood.setAngle(0);
      } else {
        // swap for solution.hoodAngle() when working
        hood.setAngle(341.5);
      }

      if (trenchUpAndMovingUp || trenchDownAndMovingDown) {
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

  private void LogTrenchAvoidence() {
    Logger.recordOutput("Controls/Trench Avoidence/closeBorder", closeBorder);
    Logger.recordOutput("Controls/Trench Avoidence/farBorder", farBorder);
    Logger.recordOutput("Controls/Trench Avoidence/rightBorder", rightBorder);
    Logger.recordOutput("Controls/Trench Avoidence/leftBorder", leftBorder);

    Logger.recordOutput("Controls/Trench Avoidence/inXRange", inXRange);
    Logger.recordOutput("Controls/Trench Avoidence/inYRange", inYRange);

    Logger.recordOutput("Controls/Trench Avoidence/trenchUpAndMovingUp", trenchUpAndMovingUp);
    Logger.recordOutput(
        "Controls/Trench Avoidence/trenchDownAndMovingDown", trenchDownAndMovingDown);
    Logger.recordOutput("Controls/Trench Avoidence/onRightAndMovingRight", onRightAndMovingRight);
    Logger.recordOutput("Controls/Trench Avoidence/onLeftAndMovingLeft", onLeftAndMovingLeft);

    Logger.recordOutput(
        "Controls/Trench Avoidence/approachingFromAllianceSide", approachingFromAllianceSide);
    Logger.recordOutput("Controls/Trench Avoidence/approachingFromRight", approachingFromRight);

    Logger.recordOutput("Controls/Trench Avoidence/myX", myX);
    Logger.recordOutput("Controls/Trench Avoidence/myY", myY);

    Logger.recordOutput("Controls/Trench Avoidence/dX", dX);
    Logger.recordOutput("Controls/Trench Avoidence/dY", dY);
  }
}
