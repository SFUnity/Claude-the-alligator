package frc.robot.subsystems.shooter.turret;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GeneralUtil;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private double targetDegs = 0;
  // private double setpointDegs = 0;
  private double lastTargetDegs = 0;
  private double targetVelocity = 0;
  private boolean isShooting = false;
  private double truePositionDegs = 0;
  private double positionDegs = 0;

  private boolean eDisabled = false; // TODO tune debouncers + their offsets
  private Debouncer outOfBoundsDebouncer = new Debouncer(0.1, DebounceType.kRising);
  private Debouncer awayFromSetpointDebouncer = new Debouncer(1.0, DebounceType.kRising);

  private LoggedTunableNumber maxVelocity = new LoggedTunableNumber("Turret/maxVelocity", 360);
  private LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Turret/maxAcceleration", 99999);

  private TrapezoidProfile profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));

  private State setpoint = new State();

  private LoggedTunableNumber kA = new LoggedTunableNumber("Turret/kA", 0.1);
  private LoggedTunableNumber kV = new LoggedTunableNumber("Turret/kV", 0.1);

  public final Alert encoder1Disconnected;
  public final Alert encoder2Disconnected;

  private final Debouncer encoder1DisconnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kRising);
  private final Debouncer encoder2DisconnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kRising);

  private final double talonOffsetRots;

  public Turret(TurretIO io) {
    this.io = io;
    io.updateInputs(inputs);
    double motorOffsetDegs = 0; // getMotorOffsetDegs(); TODO change later
    Logger.recordOutput("Shooter/Turret/MotorOffsetDegs", motorOffsetDegs);
    if (motorOffsetDegs < trueMinAngleDegs || motorOffsetDegs > trueMaxAngleDegs) {
      eDisabled = true;
    }
    talonOffsetRots = Units.degreesToRotations(motorOffsetDegs) * gearRatio - inputs.talonRotations;

    encoder1Disconnected = new Alert("Encoder 1 Disconnected!", AlertType.kWarning);
    encoder2Disconnected = new Alert("Encoder 2 Disconnected!", AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    truePositionDegs = getPositionDegs();
    positionDegs = MathUtil.inputModulus(truePositionDegs, 0, 360);

    if (maxVelocity.hasChanged(hashCode())) {
      profile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
    }
    if (maxAcceleration.hasChanged(hashCode())) {
      profile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
    }

    Logger.recordOutput("Shooter/Turret/TruePositionDegs", truePositionDegs);
    Logger.recordOutput("Shooter/Turret/PositionDegs", positionDegs);
    Logger.recordOutput("Shooter/Turret/eDisabled", eDisabled);
    Logger.processInputs("Shooter/Turret", inputs);
    GeneralUtil.logSubsystem(this, "Shooter/Turret");

    encoder1Disconnected.set(encoder1DisconnectedDebouncer.calculate(inputs.encoder1Disconnected));
    encoder2Disconnected.set(encoder2DisconnectedDebouncer.calculate(inputs.encoder2Disconnected));

    if (!eDisabled) {
      eDisabled =
          outOfBoundsDebouncer.calculate(
              (truePositionDegs < trueMinAngleDegs + 10 && inputs.velocityDegsPerSec < 0)
                  || (truePositionDegs > trueMaxAngleDegs - 10 && inputs.velocityDegsPerSec > 0));
      //|| awayFromSetpointDebouncer.calculate(Math.abs(truePositionDegs - targetDegs) > 10);

      double minLegalAngle = isShooting ? minAngleDegs : minBufferAngleDegs;
      double maxLegalAngle = isShooting ? maxAngleDegs : maxBufferAngleDegs;

      boolean hasBestAngle = false;
      double bestAngle = 0;

      for (int i = 0; i < 5; i++) {
        double potentialSetpoint = targetDegs + 360 * i;
        if (potentialSetpoint < minLegalAngle || potentialSetpoint > maxLegalAngle) {
          continue;
        } else {
          if (!hasBestAngle) {
            bestAngle = potentialSetpoint;
            hasBestAngle = true;
          }
          if (Math.abs(lastTargetDegs - potentialSetpoint) < Math.abs(lastTargetDegs - bestAngle)) {
            bestAngle = potentialSetpoint;
          }
        }
      }
      lastTargetDegs = bestAngle;

      Logger.recordOutput("Shooter/Turret/GoalAngle", bestAngle);

      State goalState =
          new State(MathUtil.clamp(bestAngle, minLegalAngle, maxLegalAngle), targetVelocity);

      setpoint = profile.calculate(loopPeriodSecs, setpoint, goalState);

      Logger.recordOutput("Shooter/Turret/SetpointAngle", setpoint.position);
      targetDegs = setpoint.position;

      double targetRotations = Units.degreesToRotations(targetDegs) * gearRatio;
      targetRotations -= talonOffsetRots;
      Logger.recordOutput("Shooter/Turret/SetpointRotations", targetRotations);
      io.turnTurret(targetRotations, setpoint.velocity, kA.get(), kV.get());
    } else {
      io.stop();
    }
  }

  // @AutoLogOutput(key = "Shooter/Turret/MotorOffsetDegs")
  public double getMotorOffsetDegs() {
    double truePosition = 0;
    double position1 = inputs.encoder1Rotations;
    double position2 = inputs.encoder2Rotations;

    position1 = Units.rotationsToDegrees(position1);
    position2 = Units.rotationsToDegrees(position2);

    // chinese remainder theorem from claude check later
    // truePosition =
    //     ((encoder1Gear ^ 2 * encoder2Gear * 360) * position1
    //             + (encoder2Gear ^ 2 * encoder1Gear * 360) * position2)
    //         % (360 ^ 2 * encoder1Gear * encoder2Gear);

    return truePosition;
  }

  public double getPositionDegs() {
    return Units.rotationsToDegrees(inputs.talonRotations + talonOffsetRots) / gearRatio;
  }

  public boolean getEDisabled() {
    return eDisabled;
  }

  public void enable() {
    eDisabled = false;
  }

  public boolean atGoal() {
    return Math.abs(positionDegs - targetDegs) < angleToleranceDegs
        && inputs.velocityDegsPerSec
            < velocityToleranceDegs; // TODO Change the velocity tolerance to be within a set
    // velocity, rather than just 0
  }

  public void setTarget(double targetDegs, double targetVelocity) {
    this.targetDegs = targetDegs;
    this.targetVelocity = targetVelocity;
  }

  public void setIsShooting(boolean isShooting) {
    this.isShooting = isShooting;
  }
}
