// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.RobotCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakePivot.IntakePivotIO;
import frc.robot.subsystems.intakePivot.IntakePivotIOSim;
import frc.robot.subsystems.intakePivot.IntakePivotIOTalon;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.rollers.intakerollers.IntakeRollers;
import frc.robot.subsystems.rollers.intakerollers.IntakeRollersIO;
import frc.robot.subsystems.rollers.intakerollers.IntakeRollersIOSim;
import frc.robot.subsystems.rollers.intakerollers.IntakeRollersIOTalonFX;
import frc.robot.subsystems.rollers.kicker.Kicker;
import frc.robot.subsystems.rollers.kicker.KickerIO;
import frc.robot.subsystems.rollers.kicker.KickerIOSim;
import frc.robot.subsystems.rollers.kicker.KickerIOTalonFX;
import frc.robot.subsystems.rollers.spindexer.Spindexer;
import frc.robot.subsystems.rollers.spindexer.SpindexerIO;
import frc.robot.subsystems.rollers.spindexer.SpindexerIOSim;
import frc.robot.subsystems.rollers.spindexer.SpindexerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheels.Flywheels;
import frc.robot.subsystems.shooter.flywheels.FlywheelsIO;
import frc.robot.subsystems.shooter.flywheels.FlywheelsIOSim;
import frc.robot.subsystems.shooter.flywheels.FlywheelsIOTalonFX;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFX;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.subsystems.shooter.turret.TurretIOTalonFX;
import frc.robot.util.FuelSim;
import frc.robot.util.PoseManager;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Spindexer spindexer;
  private final Climb climb;
  private final Flywheels flywheels;
  private final IntakePivot intakePivot;
  private final IntakeRollers intakeRollers;
  private final Turret turret;
  private final Shooter shooter;
  private final Hood hood;
  private final Kicker kicker;

  // Non-subsystems
  private final Autos autos;
  private final PoseManager poseManager = new PoseManager();

  public FuelSim fuelSim = new FuelSim("Fuel Sim");

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private boolean intakeDown = false;

  // Alerts
  private static final double canErrorTimeThreshold = 0.5; // Seconds to disable alert
  private static final double lowBatteryVoltage = 12.5;
  private static final double extraLowBatteryVoltage = 11.5;
  private static final double lowBatteryDisabledTime = 1.5;

  private final Timer disabledTimer = new Timer();
  private final Timer canInitialErrorTimer = new Timer();
  private final Timer canErrorTimer = new Timer();

  private final Alert canErrorAlert = // YAY always fun
      new Alert("CAN errors detected, robot may not be controllable.", AlertType.kError);
  private final Alert lowBatteryAlert =
      new Alert(
          "Battery voltage is very low, consider turning off the robot or replacing the battery.",
          AlertType.kWarning);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  @SuppressWarnings("resource")
  public RobotContainer() {
    // Reset alert timers
    canInitialErrorTimer.restart();
    canErrorTimer.restart();
    disabledTimer.restart();

    // Alerts for constants
    if (Constants.tuningMode) {
      new Alert("Tuning mode enabled", AlertType.kInfo).set(true);
    }

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                poseManager);
        spindexer = new Spindexer(new SpindexerIOTalonFX());
        climb = new Climb(new ClimbIOTalonFX());
        flywheels = new Flywheels(new FlywheelsIOTalonFX());
        turret = new Turret(new TurretIOTalonFX());
        hood = new Hood(new HoodIOTalonFX());
        shooter = new Shooter(flywheels, turret, hood, poseManager);
        intakePivot = new IntakePivot(new IntakePivotIOTalon());
        intakeRollers = new IntakeRollers(new IntakeRollersIOTalonFX());
        kicker = new Kicker(new KickerIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight),
                poseManager);
        spindexer = new Spindexer(new SpindexerIOSim());
        climb = new Climb(new ClimbIOSim());
        intakePivot = new IntakePivot(new IntakePivotIOSim());
        intakeRollers = new IntakeRollers(new IntakeRollersIOSim());
        flywheels = new Flywheels(new FlywheelsIOSim());
        turret = new Turret(new TurretIOSim());
        hood = new Hood(new HoodIOSim());
        kicker = new Kicker(new KickerIOSim());
        shooter = new Shooter(flywheels, turret, hood, poseManager);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                poseManager);
        spindexer = new Spindexer(new SpindexerIO() {});
        climb = new Climb(new ClimbIO() {});
        intakePivot = new IntakePivot(new IntakePivotIO() {});
        intakeRollers = new IntakeRollers(new IntakeRollersIO() {});
        flywheels = new Flywheels(new FlywheelsIO() {});
        turret = new Turret(new TurretIO() {});
        hood = new Hood(new HoodIO() {});
        kicker = new Kicker(new KickerIO() {});
        shooter = new Shooter(flywheels, turret, hood, poseManager);
        break;
    }

    autos = new Autos(drive, poseManager);

    // For tuning visualizations
    // Logger.recordOutput("ZeroedPose2d", new Pose2d());
    // Logger.recordOutput("ZeroedPose3d", new Pose3d[] {new Pose3d(), new Pose3d()});

    // Configure the button bindings
    configureButtonBindings();
  }

  public void checkAlerts() {
    // Check controllers
    boolean driverConnected =
        controller.isConnected()
            && DriverStation.getJoystickIsXbox(
                controller.getHID().getPort()); // Should be an XBox controller
    driverDisconnected.set(!driverConnected);
    Logger.recordOutput("Controls/driverConnected", driverConnected);

    // Check CAN status
    var canStatus = RobotController.getCANStatus();
    if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
      canErrorTimer.restart();
    }
    canErrorAlert.set(
        !canErrorTimer.hasElapsed(canErrorTimeThreshold)
            && canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));

    // Low battery alert
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }
    if (disabledTimer.hasElapsed(lowBatteryDisabledTime)) {
      double voltage = RobotController.getBatteryVoltage();
      if (voltage <= extraLowBatteryVoltage) {
        lowBatteryAlert.set(true);
        Leds.getInstance().extraLowBatteryAlert = true;
      } else if (voltage <= lowBatteryVoltage) {
        lowBatteryAlert.set(true);
        Leds.getInstance().lowBatteryAlert = true;
      }
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.snakeDrive(
            drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), poseManager));
    spindexer.setDefaultCommand(spindexer.stop());
    climb.setDefaultCommand(climb.climbDown());
    intakePivot.setDefaultCommand(intakePivot.raise());
    intakeRollers.setDefaultCommand(intakeRollers.stop());
    kicker.setDefaultCommand(kicker.stop());

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero,
                poseManager));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        poseManager.setPose(
                            new Pose2d(poseManager.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    controller.y().whileTrue(intakePivot.lower());

    controller.povUp().whileTrue(climb.climbUp());
    controller.povDown().whileTrue(climb.climbDown());
    // controller
    //     .leftBumper()
    //     .onTrue(
    //         Commands.sequence(
    //             Commands.runOnce(
    //                 () -> {
    //                   intakeDown = !intakeDown;
    //                   Logger.recordOutput("Intake/intakeDown", intakeDown);
    //                 }),
    //             Commands.either(
    //                 RobotCommands.intake(intakeRollers, intakePivot),
    //                 RobotCommands.stowIntake(intakeRollers, intakePivot),
    //                 () -> intakeDown)));

    controller.leftBumper().toggleOnTrue(Commands.runOnce(() -> intakeDown = !intakeDown));
    controller
        .leftBumper()
        .and(() -> intakeDown)
        .onTrue(RobotCommands.stowIntake(intakeRollers, intakePivot));
    controller
        .leftBumper()
        .and(() -> !intakeDown)
        .onTrue(RobotCommands.intake(intakeRollers, intakePivot));
    controller.rightTrigger().whileTrue(flywheels.setVelocity(1000));
    controller.leftTrigger().whileTrue(RobotCommands.jork(intakeRollers, intakePivot));
    controller.rightBumper().onTrue(spindexer.run().alongWith(kicker.run()).withName("runSpindexerAndKicker"));
    // Commands.either(
    //         RobotCommands.intake(intakeRollers, intakePivot),
    //         RobotCommands.stowIntake(intakeRollers, intakePivot),
    //         () -> {
    //           return intakeDown;
    //         })
    //     .beforeStarting(
    //         () -> {
    //           if (intakeDown == true) {
    //             intakeDown = false;
    //             Logger.recordOutput("Intake/intakeDown", intakeDown);
    //           } else if (intakeDown == false) {
    //             intakeDown = true;
    //             Logger.recordOutput("Intake/intakeDown", intakeDown);
    //           }
    //         }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autos.getAutonomousCommand();
  }
}
