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
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private double targetDegs = 0;
  private double lastTargetDegs = 0;
  private double targetVelocity = 0;
  private boolean isShooting = false;
  private double truePositionDegs = 0;
  private double positionDegs = 0;
  private double possiblePosition = 0;

  private boolean eDisabled = false; // TODO tune debouncers + their offsets
  private Debouncer outOfBoundsDebouncer =
      new Debouncer(0.1, DebounceType.kRising); // TODO add stuff for if any encoder breaks
  private Debouncer encoderBrokenDebouncer = new Debouncer(0.5, DebounceType.kRising);
  private double lastTalonRotations = 0;

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

  private double talonOffsetRots;

  private double motorOffsetDegs = -1;
  private boolean hasBeenZeroed = false;

  private final LoggedNetworkBoolean eDisabledDashboard =
      new LoggedNetworkBoolean("turn off eDisabled", false);

  public Turret(TurretIO io) {
    this.io = io;
    io.updateInputs(inputs);
    // motorOffsetDegs = getMotorOffsetDegs();
    // Logger.recordOutput("Shooter/Turret/MotorOffsetDegs", motorOffsetDegs);
    // if (motorOffsetDegs < trueMinAngleDegs || motorOffsetDegs > trueMaxAngleDegs) {
    //   eDisabled = true;
    // }
    // talonOffsetRots = Units.degreesToRotations(motorOffsetDegs) * gearRatio -
    // inputs.talonRotations;

    encoder1Disconnected = new Alert("Encoder 1 Disconnected!", AlertType.kWarning);
    encoder2Disconnected = new Alert("Encoder 2 Disconnected!", AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    if (!hasBeenZeroed || Math.abs(possiblePosition - truePositionDegs) > 5) {
      possiblePosition = getMotorOffsetDegs();
      if (possiblePosition > -5) {
        io.resetMotorEncoder(Units.degreesToRotations(possiblePosition) * gearRatio);
        hasBeenZeroed = true;
      }
    }
    truePositionDegs = getPositionDegs() / gearRatio;
    positionDegs = MathUtil.inputModulus(truePositionDegs, 0, 360);

    Logger.recordOutput("Shooter/Turret/TruePositionDegs", truePositionDegs);
    Logger.recordOutput("Shooter/Turret/PositionDegs", positionDegs);
    Logger.recordOutput("Shooter/Turret/eDisabledCantChange", eDisabled);
    Logger.recordOutput("Shooter/Turret/hasbeenZeroed", hasBeenZeroed);
    Logger.processInputs("Shooter/Turret", inputs);
    GeneralUtil.logSubsystem(this, "Shooter/Turret");

    if (truePositionDegs >= trueMinAngleDegs - 3
        && truePositionDegs <= trueMaxAngleDegs + 3
        && hasBeenZeroed) {

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

      encoder1Disconnected.set(
          encoder1DisconnectedDebouncer.calculate(inputs.encoder1Disconnected));
      encoder2Disconnected.set(
          encoder2DisconnectedDebouncer.calculate(inputs.encoder2Disconnected));

      if (eDisabledDashboard.get()) eDisabled = false;

      if (!eDisabled) {
        boolean outOfBounds =
            outOfBoundsDebouncer.calculate(
                (truePositionDegs < trueMinAngleDegs && inputs.velocityDegsPerSec < 0)
                    || (truePositionDegs > trueMaxAngleDegs && inputs.velocityDegsPerSec > 0));
        boolean encoderBroken =
            encoderBrokenDebouncer.calculate(
                Math.abs(inputs.appliedVolts) > 0.5
                    && Math.abs(inputs.talonRotations - lastTalonRotations) < 0.05);
        lastTalonRotations = inputs.talonRotations;

        Logger.recordOutput("Shooter/Turret/outOfBounds", outOfBounds);
        Logger.recordOutput("Shooter/Turret/encoderBroken", encoderBroken);

        eDisabled = outOfBounds || encoderBroken;
        
        if(eDisabled) {io.stop(); return;}

        double minLegalAngle = isShooting ? minAngleDegs : minBufferAngleDegs;
        double maxLegalAngle = isShooting ? maxAngleDegs : maxBufferAngleDegs;

        boolean hasBestAngle = false;
        double bestAngle = 0;

        // replace
        for (int i = 0; i < 5; i++) {
          double potentialSetpoint = targetDegs + 360 * i;
          if (potentialSetpoint < minLegalAngle || potentialSetpoint > maxLegalAngle) {
            continue;
          } else {
            if (!hasBestAngle) {
              bestAngle = potentialSetpoint;
              hasBestAngle = true;
            }
            if (Math.abs(lastTargetDegs - potentialSetpoint)
                < Math.abs(lastTargetDegs - bestAngle)) {
              bestAngle = potentialSetpoint;
            }
          }
        }
        lastTargetDegs = bestAngle;

        Logger.recordOutput("Shooter/Turret/GoalAngle", bestAngle);

        // State goalState =
        //     new State(MathUtil.clamp(bestAngle, minLegalAngle, maxLegalAngle), targetVelocity);

        // setpoint = profile.calculate(loopPeriodSecs, setpoint, goalState);

        // Logger.recordOutput("Shooter/Turret/SetpointAngle", setpoint.position);
        targetDegs = bestAngle;

        double targetRotations = Units.degreesToRotations(targetDegs) * gearRatio;
        // targetRotations -= talonOffsetRots;
        Logger.recordOutput("Shooter/Turret/SetpointRotations", targetRotations);
        io.turnTurret(targetRotations);
      } else {
        io.stop();
      }
    }
  }

  public double getMotorOffsetDegsTestable(double position1, double position2) {

    int maxEnc1Rotations = (int) Math.ceil(turretGear / encoder1Gear); // = 8

    for (int i = 0; i < maxEnc1Rotations; i++) {
      // Candidate: enc1 is on its i-th full rotation
      // Turret position in turret-rotations
      double candidateTurretPos = (position1 + i) * encoder1Gear / turretGear;
      // Simplifies to:
      // double candidateTurretPos = (enc1 + i) / encoder1Gear;

      // What would enc2 read at this turret position?
      double expectedEnc2 = (candidateTurretPos * turretGear / encoder2Gear) % 1.0;

      if (Math.abs(expectedEnc2 - position2) < 0.01) {
        return Units.rotationsToDegrees(
            candidateTurretPos); // in turret rotations, multiply by 360 for degrees
      }
    }
    return -100;
  }

  public double getMotorOffsetDegs() {
    double position1 = MathUtil.inputModulus(inputs.encoder1Rotations, 0, 1);
    double position2 = MathUtil.inputModulus(inputs.encoder2Rotations, 0, 1);
    return getMotorOffsetDegsTestable(position1, position2);
  }

  public double getPositionDegs() {
    return Units.rotationsToDegrees(inputs.talonRotations);
  }

  public boolean getEDisabled() {
    return eDisabled;
  }

  public void enable() {
    eDisabled = false;
  }

  @AutoLogOutput(key = "Shooter/Turret/AtGoal")
  public boolean atGoal() {
    return Math.abs(positionDegs - targetDegs) < angleToleranceDegs
        && inputs.velocityDegsPerSec
            < velocityToleranceDegs; // TODO Change the velocity tolerance to be within a set
    // velocity, rather than just 0
  }

  public void setTarget(double targetDegs) {
    this.targetDegs = MathUtil.inputModulus(targetDegs + turretOffsetDegs, 0, 360);
  }

  public void setIsShooting(boolean isShooting) {
    this.isShooting = isShooting;
  }
}
