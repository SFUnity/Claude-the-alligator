package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.RobotCommands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.rollers.intakerollers.IntakeRollers;
import frc.robot.subsystems.rollers.kicker.Kicker;
import frc.robot.subsystems.rollers.spindexer.Spindexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.util.LoggedAutoChooser;
import frc.robot.util.PoseManager;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {
  private final Drive drive;
  private final PoseManager poseManager;
  private final IntakeRollers intake;
  private final IntakePivot intakePivot;
  private final Shooter shooter;
  private final Kicker kicker;
  private final Spindexer spindexer;
  private final Climb climb;
  private final Hood hood;

  private final AutoFactory factory;
  private final LoggedAutoChooser chooser;

  private final LoggedDashboardChooser<Command> nonChoreoChooser =
      new LoggedDashboardChooser<Command>("Non-Choreo Chooser");
  private static final boolean isChoreoAuto = true;

  public static boolean moveRight = false;
  public static boolean moveLeft = false;

  public Autos(
      Drive drive,
      PoseManager poseManager,
      IntakeRollers intake,
      IntakePivot intakePivot,
      Shooter shooter,
      Kicker kicker,
      Spindexer spindexer,
      Climb climb,
      Hood hood) {
    this.drive = drive;
    this.poseManager = poseManager;
    this.intake = intake;
    this.intakePivot = intakePivot;
    this.shooter = shooter;
    this.kicker = kicker;
    this.spindexer = spindexer;
    this.climb = climb;
    this.hood = hood;

    factory =
        new AutoFactory(
            poseManager::getPose,
            poseManager::setPose,
            drive::followTrajectory,
            true,
            drive,
            (Trajectory<SwerveSample> traj, Boolean bool) -> {
              //   Logger.recordOutput(
              //       "Drive/Choreo/Active Traj",
              //       (AllianceFlipUtil.shouldFlip() ? traj.flipped() : traj).getPoses());
              //   Logger.recordOutput(
              //       "Drive/Choreo/Current Traj End Pose",
              //       traj.getFinalPose(AllianceFlipUtil.shouldFlip()).get());
              //   Logger.recordOutput(
              //       "Drive/Choreo/Current Traj Start Pose",
              //       traj.getInitialPose(AllianceFlipUtil.shouldFlip()).get());
            });

    /* Set up main choreo routines */
    chooser = new LoggedAutoChooser("/ChoreoChooser");
    // chooser.addRoutine("Example Auto Routine", this::exampleAutoRoutine);

    if (!DriverStation.isFMSAttached()) {
      // Set up test choreo routines

      // SysID & non-choreo routines
      if (!isChoreoAuto) {
        nonChoreoChooser.addOption("TuneModuleDrive", DriveCommands.tuneModuleDrive(drive));
        nonChoreoChooser.addOption("TuneModuleTurn", DriveCommands.tuneModuleTurn(drive));
        // Set up SysId routines
        nonChoreoChooser.addOption(
            "Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization(drive, poseManager));
        nonChoreoChooser.addOption(
            "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        nonChoreoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        nonChoreoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        nonChoreoChooser.addOption(
            "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        nonChoreoChooser.addOption(
            "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      }
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return isChoreoAuto ? chooser.selectedCommandScheduler() : nonChoreoChooser.get();
  }

  public AutoRoutine UpperCollectAndShoot() {
    AutoRoutine routine = factory.newRoutine("UpperCollectAndShoot");
    AutoTrajectory UpperCollectAndShoot = routine.trajectory("UpperCollectAndShoot");
    routine
        .active()
        .onTrue(
            Commands.sequence(
                UpperCollectAndShoot.resetOdometry(),
                Commands.waitSeconds(5),
                UpperCollectAndShoot.cmd()));
    UpperCollectAndShoot.atTime("intake").onTrue(RobotCommands.intake(intake, intakePivot));
    UpperCollectAndShoot.atTime("Shoot")
        .onTrue(RobotCommands.readyThenShoot(shooter, kicker, spindexer, drive));
    return routine;
  }

  public AutoRoutine DepotCollectAndShoot() {
    AutoRoutine routine = factory.newRoutine("DepotCollectAndShoot");
    AutoTrajectory DepotCollectAndShoot = routine.trajectory("DepotCollectAndShoot");
    routine
        .active()
        .onTrue(
            Commands.sequence(
                DepotCollectAndShoot.resetOdometry(),
                Commands.waitSeconds(5),
                DepotCollectAndShoot.cmd()));
    DepotCollectAndShoot.atTime("intake").onTrue(RobotCommands.intake(intake, intakePivot));
    DepotCollectAndShoot.atTime("shoot")
        .onTrue(RobotCommands.readyThenShoot(shooter, kicker, spindexer, drive));
    return routine;
  }
}
