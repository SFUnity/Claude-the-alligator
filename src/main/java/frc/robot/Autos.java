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
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedAutoChooser;
import frc.robot.util.PoseManager;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {
  private final Drive drive;
  private final PoseManager poseManager;

  private final AutoFactory factory;
  private final LoggedAutoChooser chooser;

  private final LoggedDashboardChooser<Command> nonChoreoChooser =
      new LoggedDashboardChooser<Command>("Non-Choreo Chooser");
  private static final boolean isChoreoAuto = true;

  public static boolean moveRight = false;
  public static boolean moveLeft = false;

  public Autos(Drive drive, PoseManager poseManager) {
    this.drive = drive;
    this.poseManager = poseManager;

    factory =
        new AutoFactory(
            poseManager::getPose,
            poseManager::setPose,
            drive::followTrajectory,
            true,
            drive,
            (Trajectory<SwerveSample> traj, Boolean bool) -> {
              Logger.recordOutput(
                  "Drive/Choreo/Active Traj",
                  (AllianceFlipUtil.shouldFlip() ? traj.flipped() : traj).getPoses());
              Logger.recordOutput(
                  "Drive/Choreo/Current Traj End Pose",
                  traj.getFinalPose(AllianceFlipUtil.shouldFlip()).get());
              Logger.recordOutput(
                  "Drive/Choreo/Current Traj Start Pose",
                  traj.getInitialPose(AllianceFlipUtil.shouldFlip()).get());
            });

    /* Set up main choreo routines */
    chooser = new LoggedAutoChooser("ChoreoChooser");
    // chooser.addRoutine("Example Auto Routine", this::exampleAutoRoutine);
    chooser.addRoutine("Climb Auto Routine", this::climbAutoRoutine);
    chooser.addRoutine("Outpost Climb Auto Routine", this::outpostClimbAutoRoutine);
    chooser.addRoutine("Depot Auto Routine", this::depotAutoRoutine);
    chooser.addRoutine("Score Center Climb Auto Routine", this::ScoreCenterClimbAutoRoutine);
    chooser.addRoutine("Score Center Climb2 Auto Routine", this::ScoreCenterClimb2AutoRoutine);
    chooser.addRoutine("Depot Feed Auto Routine", this::depotFeedAutoRoutine);
    chooser.addRoutine("Upper Feed Climb Auto Routine", this::upperFeedClimbAutoRoutine);
    chooser.addRoutine("Feed Auto Routine", this::FeedAutoRoutine);
    chooser.addRoutine("Lower Feed Auto Routine", this::LowerFeedAutoRoutine);
    chooser.addRoutine("Lower Feed Climb Auto Routine", this::LowerFeedClimbAutoRoutine);
    if (!DriverStation.isFMSAttached()) {
      // Set up test choreo routines

      // SysID & non-choreo routines
      if (!isChoreoAuto) {
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

  public AutoRoutine climbAutoRoutine() {

    AutoRoutine routine = factory.newRoutine("Climb Auto Routine");
    AutoTrajectory Climb = routine.trajectory("Climb");
    routine.active().onTrue(Commands.sequence(Climb.resetOdometry(), Climb.cmd()));
    Climb.atTime("ExtendClimber").onTrue(RobotCommands.climbExtend());
    Climb.done().onTrue(RobotCommands.climbRetract());
    return routine;
  }

  public AutoRoutine outpostClimbAutoRoutine() {
    AutoRoutine routine = factory.newRoutine("Outpost Climb Auto Routine");
    AutoTrajectory OutpostClimb = routine.trajectory("OutpostClimb");
    routine.active().onTrue(Commands.sequence(OutpostClimb.resetOdometry(), OutpostClimb.cmd()));
    OutpostClimb.atTime("ExtendClimber").onTrue(RobotCommands.climbExtend());
    OutpostClimb.done().onTrue(RobotCommands.climbRetract());
    return routine;
  }

  public AutoRoutine depotAutoRoutine() {
    AutoRoutine routine = factory.newRoutine("Depot Auto Routine");
    AutoTrajectory Depot = routine.trajectory("DepotClimb");
    routine.active().onTrue(Commands.sequence(Depot.resetOdometry(), Depot.cmd()));
    Depot.atTime("ExtendClimber").onTrue(RobotCommands.climbExtend());
    Depot.atTime("StartIntake").onTrue(RobotCommands.intake().until(Depot.atTime("StopIntake")));
    Depot.atTime("StartShoot").onTrue(RobotCommands.shoot().until(Depot.atTime("StopShoot")));
    Depot.done().onTrue(RobotCommands.climbRetract());
    return routine;
  }

  public AutoRoutine ScoreCenterClimbAutoRoutine() {
    AutoRoutine routine = factory.newRoutine("ScoreCenterClimb Auto Routine");
    AutoTrajectory ScoreCenterClimb = routine.trajectory("ScoreCenterClimb");
    routine
        .active()
        .onTrue(Commands.sequence(ScoreCenterClimb.resetOdometry(), ScoreCenterClimb.cmd()));
    ScoreCenterClimb.atTime("ExtendClimber").onTrue(RobotCommands.climbExtend());
    ScoreCenterClimb.atTime("StartIntake")
        .onTrue(RobotCommands.intake().until(ScoreCenterClimb.atTime("StopIntake")));
    ScoreCenterClimb.atTime("StartShoot")
        .onTrue(RobotCommands.shoot().until(ScoreCenterClimb.atTime("StopShoot")));
    ScoreCenterClimb.done().onTrue(RobotCommands.climbRetract());
    return routine;
  }

  public AutoRoutine ScoreCenterClimb2AutoRoutine() {
    AutoRoutine routine = factory.newRoutine("ScoreCenterClimb2 Auto Routine");
    AutoTrajectory ScoreCenterClimb2 = routine.trajectory("ScoreCenterClimb2");
    routine
        .active()
        .onTrue(Commands.sequence(ScoreCenterClimb2.resetOdometry(), ScoreCenterClimb2.cmd()));
    return routine;
  }

  public AutoRoutine depotFeedAutoRoutine() {
    AutoRoutine routine = factory.newRoutine("Depot Feed Auto Routine");
    AutoTrajectory DepotFeed = routine.trajectory("DepotFeedClimb");
    routine.active().onTrue(Commands.sequence(DepotFeed.resetOdometry(), DepotFeed.cmd()));
    DepotFeed.atTime("StartIntake")
        .onTrue(RobotCommands.intake().until(DepotFeed.atTime("StopIntake")));
    DepotFeed.atTime("StartShoot")
        .onTrue(RobotCommands.shoot().until(DepotFeed.atTime("StopShoot")));
    DepotFeed.atTime("StartDepotIntake")
        .onTrue(RobotCommands.intake().until(DepotFeed.atTime("StopDepotIntake")));
    DepotFeed.atTime("ExtendClimber").onTrue(RobotCommands.climbExtend());
    DepotFeed.done().onTrue(RobotCommands.climbRetract());
    return routine;
  }

  public AutoRoutine upperFeedClimbAutoRoutine() {
    AutoRoutine routine = factory.newRoutine("Upper Feed Climb Auto Routine");
    AutoTrajectory UpperFeedClimb = routine.trajectory("UpperFeedClimb");
    routine
        .active()
        .onTrue(Commands.sequence(UpperFeedClimb.resetOdometry(), UpperFeedClimb.cmd()));
    UpperFeedClimb.atTime("StartIntake")
        .onTrue(RobotCommands.intake().until(UpperFeedClimb.atTime("StopIntake")));
    UpperFeedClimb.atTime("StartShoot")
        .onTrue(RobotCommands.shoot().until(UpperFeedClimb.atTime("StopShoot")));
    UpperFeedClimb.atTime("ExtendClimber").onTrue(RobotCommands.climbExtend());
    UpperFeedClimb.done().onTrue(RobotCommands.climbRetract());
    return routine;
  }

  public AutoRoutine FeedAutoRoutine() {
    AutoRoutine routine = factory.newRoutine("Feed Auto Routine");
    AutoTrajectory Feed = routine.trajectory("Feed");
    routine.active().onTrue(Commands.sequence(Feed.resetOdometry(), Feed.cmd()));
    Feed.atTime("StartIntake").onTrue(RobotCommands.intake().until(Feed.atTime("StopIntake")));
    Feed.atTime("StartShooting").onTrue(RobotCommands.shoot().until(Feed.atTime("StopShooting")));
    return routine;
  }

  public AutoRoutine LowerFeedAutoRoutine() {
    AutoRoutine routine = factory.newRoutine("Lower Feed Auto Routine");
    AutoTrajectory LowerFeed = routine.trajectory("LowerFeed");
    routine.active().onTrue(Commands.sequence(LowerFeed.resetOdometry(), LowerFeed.cmd()));
    LowerFeed.atTime("StartIntake")
        .onTrue(RobotCommands.intake().until(LowerFeed.atTime("StopIntake")));
    LowerFeed.atTime("StartShooting")
        .onTrue(RobotCommands.shoot().until(LowerFeed.atTime("StopShooting")));
    return routine;
  }

  public AutoRoutine LowerFeedClimbAutoRoutine() {
    AutoRoutine routine = factory.newRoutine("Lower Feed Climb Auto Routine");
    AutoTrajectory LowerFeedClimb = routine.trajectory("LowerFeedClimb");
    routine
        .active()
        .onTrue(Commands.sequence(LowerFeedClimb.resetOdometry(), LowerFeedClimb.cmd()));
    LowerFeedClimb.atTime("StartIntake")
        .onTrue(RobotCommands.intake().until(LowerFeedClimb.atTime("StopIntake")));
    LowerFeedClimb.atTime("StartShooting")
        .onTrue(RobotCommands.shoot().until(LowerFeedClimb.atTime("StopShooting")));
    LowerFeedClimb.atTime("ExtendClimber").onTrue(RobotCommands.climbExtend());
    LowerFeedClimb.done().onTrue(RobotCommands.climbRetract());
    return routine;
  }
}
