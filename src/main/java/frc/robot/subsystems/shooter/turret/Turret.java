package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  private double targetVelocity = 0;
  private boolean atTarget = false;
  private boolean isShooting = false;
  private double truePositionDegs = 0;
  private double positionDegs = 0;

  private LoggedTunableNumber maxVelocity = new LoggedTunableNumber("Turret/maxVelocity", 16);
  private LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Turret/maxAcceleration", 99999);

  private TrapezoidProfile profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));

  private LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP", 0.5);
  private LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD", 0.5);

  public final Alert encoder1Disconnected;
  public final Alert encoder2Disconnected;

  private final Debouncer encoder1DisconnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kRising);
  private final Debouncer encoder2DisconnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kRising);

  private final double talonOffset;

  public Turret(TurretIO io) {
    this.io = io;
    io.updateInputs(inputs);
    double motorOffset = getMotorOffset();
    talonOffset = Units.degreesToRotations(motorOffset) * gearRatio - inputs.talonRotations;

    encoder1Disconnected = new Alert("Encoder 1 Disconnected!", AlertType.kWarning);
    encoder2Disconnected = new Alert("Encoder 2 Disconnected!", AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    truePositionDegs = inputs.talonRotations;
    positionDegs = (truePositionDegs - (2 * bufferDegs)) % 360;

    Logger.recordOutput("Turret/TruePositionDegs", truePositionDegs);
    Logger.recordOutput("Turret/PositionDegs", positionDegs);
    Logger.processInputs("Turret", inputs);
    GeneralUtil.logSubsystem(this, "Turret");

    encoder1Disconnected.set(encoder1DisconnectedDebouncer.calculate(inputs.encoder1Disconnected));
    encoder2Disconnected.set(encoder2DisconnectedDebouncer.calculate(inputs.encoder2Disconnected));

    if (isShooting) {
      targetDegs += bufferDegs * 2;
      if (Math.abs(targetDegs + 360 - truePositionDegs) < Math.abs(targetDegs - truePositionDegs)
          && (targetDegs + 360) <= maxAngleDegs) {
        targetDegs += 360;
      } else if (Math.abs(targetDegs - 360 - truePositionDegs)
              < Math.abs(targetDegs - truePositionDegs)
          && (targetDegs - 360) >= minAngleDegs) {
        targetDegs -= 360;
      }
    } else {
      targetDegs += bufferDegs * 2;
    }
    double targetRotations = Units.degreesToRotations(targetDegs) * gearRatio;
    targetRotations -= talonOffset;
    io.turnTurret(targetRotations, isShooting);
  }

  public double getMotorOffset() {
    double truePosition = 0;
    double position1 = inputs.encoder1Rotations;
    double position2 = inputs.encoder2Rotations;

    position1 = Units.rotationsToDegrees(position1);
    position2 = Units.rotationsToDegrees(position2);

    // chinese remainder theorem from claude check later
    truePosition =
        ((encoder1Gear ^ 2 * encoder2Gear * 360) * position1
                + (encoder2Gear ^ 2 * encoder1Gear * 360) * position2)
            % (360 ^ 2 * encoder1Gear * encoder2Gear);

    return truePosition;
  }

  public double getPositionDegs() {
    return (inputs.talonRotations + talonOffset) / gearRatio;
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
