// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.EqualsUtil.EqualsUtil;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants.DriveCommandsConfig;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PoseManager;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {

  public static final double MAX_LINEAR_VELOCITY = Units.feetToMeters(14.5); // 6328 uses 15 ft/s
  public static final double MAX_LINEAR_ACCELERATION =
      Units.feetToMeters(75.0); // This is what 6328
  private static final Module[] modules = new Module[4];
  // Commands stuff
  private DriveCommandsConfig config;
  private static PoseManager poseManager = new PoseManager();
  private Translation2d lastSetpointTranslation;
  private static final LoggedTunableNumber linearkP =
      new LoggedTunableNumber("Drive/Commands/Linear/kP", 3.5);
  private static final LoggedTunableNumber linearkD =
      new LoggedTunableNumber("Drive/Commands/Linear/kD", 0.0);
  private static final LoggedTunableNumber thetakP =
      new LoggedTunableNumber("Drive/Commands/Theta/kP", 6.0);
  private static final LoggedTunableNumber thetakD =
      new LoggedTunableNumber("Drive/Commands/Theta/D", 0.0);
  private static final LoggedTunableNumber linearTolerance =
      new LoggedTunableNumber("Drive/Commands/Linear/controllerTolerance", 0.08);
  private static final LoggedTunableNumber maxLinearVelocity =
      new LoggedTunableNumber(
          "Drive/Commands/Linear/maxVelocity", DriveCommands.MAX_LINEAR_VELOCITY);
  private static final LoggedTunableNumber maxLinearAcceleration =
      new LoggedTunableNumber(
          "Drive/Commands/Linear/maxAcceleration", DriveCommands.MAX_LINEAR_ACCELERATION * 0.4);
  private static final LoggedTunableNumber partialAutoLinearkP =
      new LoggedTunableNumber("Drive/Commands/partialAutoLinearkP", 1.0);
  private static final LoggedTunableNumber ffMinRadius =
      new LoggedTunableNumber("AutoAlign/ffMinRadius", 0.2);
  private static final LoggedTunableNumber ffMaxRadius =
      new LoggedTunableNumber("AutoAlign/ffMaxRadius", 0.8);

  private Timer joystickInterruptTimer = new Timer();
  private LoggedTunableNumber joystickInterruptDelay =
      new LoggedTunableNumber("Drive/JoystickInterruptDelay", 1);

  private Pose2d getAngularVelocityFromProfiledPID(double targetPose) {
    return null;
    // idk bro what am I doing
  }

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          thetakP.get(), 0, thetakD.get(), new TrapezoidProfile.Constraints(0.0, 0.0));
  private final ProfiledPIDController linearController =
      new ProfiledPIDController(
          linearkP.get(), 0, linearkD.get(), new TrapezoidProfile.Constraints(0, 0));

  private Translation2d getLinearVelocityFromProfiledPID(Pose2d targetPose) {
    double currentDistance = poseManager.getDistanceTo(targetPose);
    double ffScaler =
        MathUtil.clamp(
            (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
            0.0,
            0.5);

    // Reset the linear controller
    linearController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        linearController.getSetpoint().velocity);

    // Calculate linear speed
    double driveVelocityScalar =
        linearController.getSetpoint().velocity * ffScaler
            + linearController.calculate(currentDistance, 0.0);

    if (linearAtGoal()) driveVelocityScalar = 0.0;

    lastSetpointTranslation =
        new Pose2d(targetPose.getTranslation(), poseManager.getHorizontalAngleTo(targetPose))
            .transformBy(GeomUtil.toTransform2d(linearController.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate angle to target then transform by velocity scalar
    Translation2d driveVelocity =
        new Pose2d(
                new Translation2d(),
                poseManager.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
            .getTranslation();

    return driveVelocity;
  }

  public static void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveCommands.MAX_LINEAR_VELOCITY);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      // optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("Odometry/SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("Odometry/SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  // Constants
  private static final double DEADBAND = 0.1;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  // Drive command PID tunables
  private static final LoggedTunableNumber angleKp =
      new LoggedTunableNumber("Drive/Commands/Angle/kP", 5.0);
  private static final LoggedTunableNumber angleKd =
      new LoggedTunableNumber("Drive/Commands/Angle/kD", 0.4);
  private static final LoggedTunableNumber thetaToleranceDeg =
      new LoggedTunableNumber("Drive/Commands/Angle/toleranceDeg", 2.0);

  // Drive command constraints
  private static final LoggedTunableNumber maxAngularVelocity =
      new LoggedTunableNumber("Drive/Commands/Angle - maxVelocity", 8);
  private static final LoggedTunableNumber maxAngularAcceleration =
      new LoggedTunableNumber("Drive/Commands/Angle - maxAcceleration", 20);

  // Create PID controller
  private static final ProfiledPIDController angleController =
      new ProfiledPIDController(
          angleKp.get(),
          0.0,
          angleKd.get(),
          new TrapezoidProfile.Constraints(maxAngularVelocity.get(), maxAngularAcceleration.get()));

  static {
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(Units.degreesToRadians(thetaToleranceDeg.get()));
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  public void stop() {
    runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  private boolean noJoystickInput() {
    if (!joystickInterruptTimer.hasElapsed(joystickInterruptDelay.get())) {
      return true;
    }
    return MathUtil.applyDeadband(config.getOmegaInput(), 0.2) == 0
        && MathUtil.applyDeadband(Math.hypot(config.getXInput(), config.getYInput()), 0.2) == 0
        && !config.povDownPressed()
        && !config.povUpPressed()
        && !config.povLeftPressed()
        && !config.povRightPressed();
  }

  private double linearBlending(
      double distance,
      double maxDistance,
      double manualVelocity,
      double autoVelocity,
      String axis) {
    double error = Math.min(Math.abs(distance) / partialAutoLinearkP.get(), maxDistance);
    double P = error / maxDistance;
    double manualFinal = manualVelocity * P;
    double autoFinal = autoVelocity * (1 - P);

    // Logger.recordOutput("Controls/partialAuto/"+axis+"P", P);
    // Logger.recordOutput("Controls/partialAuto/manual"+axis, manualFinal);
    // Logger.recordOutput("Controls/partialAuto/auto"+axis, autoFinal);

    return manualFinal + autoFinal;
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      PoseManager poseManager) {
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Apply rotation deadband
              double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

              // Square rotation value for more precise control
              omega = Math.copySign(omega * omega, omega);

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega * drive.getMaxAngularSpeedRadPerSec());
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? poseManager.getRotation().plus(new Rotation2d(Math.PI))
                          : poseManager.getRotation()));
            },
            drive)
        .withName("joystickDrive");
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier,
      PoseManager poseManager) {

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      poseManager.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? poseManager.getRotation().plus(new Rotation2d(Math.PI))
                          : poseManager.getRotation()));

              // Record outputs for logging
              Logger.recordOutput("Commands/angleSetpointRad", rotationSupplier.get().getRadians());
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(poseManager.getRotation().getRadians()))
        .withName("joystickDriveAtAngle");
  }

  public static Command snakeDrive(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, PoseManager poseManager) {
    return joystickDriveAtAngle(
        drive,
        () -> {
          if (new Rotation2d(poseManager.getFieldVelocity().dx, poseManager.getFieldVelocity().dy)
                  .minus(poseManager.getRotation())
                  .getDegrees()
              < 45) {
            return xSupplier.getAsDouble();
          } else {
            return 0.0;
          }
        },
        () -> {
          if (new Rotation2d(poseManager.getFieldVelocity().dx, poseManager.getFieldVelocity().dy)
                  .minus(poseManager.getRotation())
                  .getDegrees()
              < 45) {
            return ySupplier.getAsDouble();
          } else {
            return 0.0;
          }
        },
        () -> {
          if (xSupplier.getAsDouble() == 0 && ySupplier.getAsDouble() == 0) {
            return poseManager.getRotation();
          } else {
            return new Rotation2d(
                poseManager.getFieldVelocity().dx, poseManager.getFieldVelocity().dy);
          }
        },
        poseManager);
  }

  private Translation2d getLinearVelocityFromJoysticks() {
    // Convert to doubles
    double x = config.getXInput();
    double y = config.getYInput();

    // Check for slow mode
    if (config.slowMode().getAsBoolean()) {
      double multiplier = config.slowDriveMultiplier().get();
      x *= multiplier;
      y *= multiplier;
    }

    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square values and scale to max velocity
    linearMagnitude = linearMagnitude * linearMagnitude;
    linearMagnitude *= maxSpeedMetersPerSec;

    // Calcaulate new linear velocity
    Translation2d linearVelocity = new Translation2d(linearMagnitude, linearDirection);

    return linearVelocity;
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
            // Reset data
            Commands.runOnce(
                () -> {
                  velocitySamples.clear();
                  voltageSamples.clear();
                }),

            // Allow modules to orient
            Commands.run(
                    () -> {
                      drive.runCharacterization(0.0);
                    },
                    drive)
                .withTimeout(FF_START_DELAY),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(
                    () -> {
                      double voltage = timer.get() * FF_RAMP_RATE;
                      drive.runCharacterization(voltage);
                      velocitySamples.add(drive.getFFCharacterizationVelocity());
                      voltageSamples.add(voltage);
                    },
                    drive)

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      int n = velocitySamples.size();
                      double sumX = 0.0;
                      double sumY = 0.0;
                      double sumXY = 0.0;
                      double sumX2 = 0.0;
                      for (int i = 0; i < n; i++) {
                        sumX += velocitySamples.get(i);
                        sumY += voltageSamples.get(i);
                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                      }
                      double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                      double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                      NumberFormat formatter = new DecimalFormat("#0.00000");
                      System.out.println("********** Drive FF Characterization Results **********");
                      System.out.println("\tkS: " + formatter.format(kS));
                      System.out.println("\tkV: " + formatter.format(kV));
                    }))
        .withName("feedforwardCharacterization");
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive, PoseManager poseManager) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                // Reset acceleration limiter
                Commands.runOnce(
                    () -> {
                      limiter.reset(0.0);
                    }),

                // Turn in place, accelerating up to full speed
                Commands.run(
                    () -> {
                      double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                      drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                    },
                    drive)),

            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),

                // Record starting measurement
                Commands.runOnce(
                    () -> {
                      state.positions = drive.getWheelRadiusCharacterizationPositions();
                      state.lastAngle = poseManager.getRotation();
                      state.gyroDelta = 0.0;
                    }),

                // Update gyro delta
                Commands.run(
                        () -> {
                          var rotation = poseManager.getRotation();
                          state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                          state.lastAngle = rotation;
                        })

                    // When cancelled, calculate and print results
                    .finallyDo(
                        () -> {
                          double[] positions = drive.getWheelRadiusCharacterizationPositions();
                          double wheelDelta = 0.0;
                          for (int i = 0; i < 4; i++) {
                            wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                          }
                          double wheelRadius =
                              (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                          NumberFormat formatter = new DecimalFormat("#0.000");
                          System.out.println(
                              "********** Wheel Radius Characterization Results **********");
                          System.out.println(
                              "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                          System.out.println(
                              "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                          System.out.println(
                              "\tWheel Radius: "
                                  + formatter.format(wheelRadius)
                                  + " meters, "
                                  + formatter.format(Units.metersToInches(wheelRadius))
                                  + " inches");
                        })))
        .withName("wheelRadiusCharacterization");
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }

  private void updateTunables() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> linearController.setPID(linearkP.get(), 0, linearkD.get()),
        linearkP,
        linearkD);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> linearController.setTolerance(linearTolerance.get()), linearTolerance);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> linearController.setPID(linearkP.get(), 0, linearkD.get()),
        linearkP,
        linearkD);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> linearController.setTolerance(linearTolerance.get()), linearTolerance);

    LoggedTunableNumber.ifChanged(
        hashCode(), this::updateConstraints, maxLinearVelocity, maxLinearAcceleration);
    updateThetaTunables();
  }

  private void updateThetaTunables() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> thetaController.setPID(thetakP.get(), 0, thetakD.get()),
        thetakP,
        thetakD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> thetaController.setTolerance(Units.degreesToRadians(thetaToleranceDeg.get())),
        thetaToleranceDeg);
    LoggedTunableNumber.ifChanged(
        hashCode(), this::updateThetaConstraints, maxAngularVelocity, maxAngularAcceleration);
  }

  private void updateConstraints() {
    linearController.setConstraints(
        new TrapezoidProfile.Constraints(maxLinearVelocity.get(), maxLinearAcceleration.get()));
    updateThetaConstraints();
  }

  private void updateThetaConstraints() {
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(maxAngularVelocity.get(), maxAngularAcceleration.get()));
  }

  private void resetControllers(Pose2d goalPose) {
    Twist2d fieldVelocity = poseManager.fieldVelocity();
    double linearVelocity =
        Math.min(
            0.0,
            new Translation2d(fieldVelocity.dx, fieldVelocity.dy)
                .rotateBy(poseManager.getHorizontalAngleTo(goalPose))
                .getX());
    linearController.reset(poseManager.getDistanceTo(goalPose), linearVelocity);
    resetThetaController();
  }

  private void resetThetaController() {
    Pose2d currentPose = poseManager.getPose();
    Twist2d fieldVelocity = poseManager.fieldVelocity();
    thetaController.reset(currentPose.getRotation().getRadians(), fieldVelocity.dtheta);
  }

  /** Returns true if within tolerance of aiming at goal */
  @AutoLogOutput(key = "Drive/Commands/Linear/AtGoal")
  public boolean linearAtGoal() {
    return linearController.atGoal();
  }

  /** Returns true if within tolerance of aiming at speaker */
  @AutoLogOutput(key = "Drive/Commands/Theta/AtGoal")
  public boolean thetaAtGoal() {
    return EqualsUtil.equalsWithTolerance(
        thetaController.getSetpoint().position,
        thetaController.getGoal().position,
        Units.degreesToRadians(thetaToleranceDeg.get()));
  }

  public Command fullAutoDrive(Supplier<Pose2d> goalPose) {
    return Commands.run(
            () -> {
              updateTunables();
              updateConstraints();

              Pose2d targetPose = goalPose.get();

              // Calculate linear speed
              Translation2d driveVelocity = getLinearVelocityFromProfiledPID(targetPose);

              // Calculate theta speed
              Pose2d thetaVelocity =
                  getAngularVelocityFromProfiledPID(targetPose.getRotation().getRadians());

              // Send command
              runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      driveVelocity.getX(),
                      driveVelocity.getY(),
                      thetaVelocity.getRotation().getRadians(),
                      poseManager.getRotation()));
            })
        .beforeStarting(
            () -> {
              joystickInterruptTimer.restart();
              resetControllers(goalPose.get());
            })
        .finallyDo(
            () -> {
              stop();
            })
        .onlyWhile(this::noJoystickInput)
        .withName("Full Auto Drive");
  }

  public Command partialAutoDrive(Supplier<Pose2d> goalPose) {
    return Commands.run(
            () -> {
              // Get manual linear velocity
              Translation2d manualLinearVelocity = getLinearVelocityFromJoysticks();
              Translation2d flippedManualLinearVelocity =
                  AllianceFlipUtil.shouldFlip()
                      ? manualLinearVelocity.rotateBy(new Rotation2d(Math.PI))
                      : manualLinearVelocity;

              // Auto-align section
              updateTunables();
              updateConstraints();

              Pose2d targetPose = goalPose.get();

              Translation2d driveVelocity = getLinearVelocityFromProfiledPID(targetPose);

              // Calculate theta speed
              Pose2d thetaVelocity =
                  getAngularVelocityFromProfiledPID(targetPose.getRotation().getRadians());

              if (thetaController.atGoal()) thetaVelocity = new Pose2d(0, 0, new Rotation2d(0));
              // Send command
              double maxDistance = 2;
              Translation2d distance =
                  poseManager.getTranslation().minus(goalPose.get().getTranslation());

              // Linear blending
              double finalX =
                  linearBlending(
                      distance.getX(),
                      maxDistance,
                      flippedManualLinearVelocity.getX(),
                      driveVelocity.getX(),
                      "X");
              double finalY =
                  linearBlending(
                      distance.getY(),
                      maxDistance,
                      flippedManualLinearVelocity.getY(),
                      driveVelocity.getY(),
                      "Y");

              runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      finalX, finalY, 0, poseManager.getRotation()));
            })
        .beforeStarting(() -> resetControllers(goalPose.get()))
        .withName("Partial Auto Drive");}
  public static Command tuneModuleTurn(Drive drive) {
    return Commands.startEnd(drive::tuneModuleTurn, drive::endTuneModule, drive)
        .withTimeout(5.0)
        .withName("tuneModuleTurn");
  }

  public static Command tuneModuleDrive(Drive drive) {
    return Commands.startEnd(drive::tuneModuleDrive, drive::endTuneModule, drive)
        .withTimeout(2.0)
        .withName("tuneModuleDrive");
  }
}
