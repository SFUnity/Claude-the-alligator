package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GeneralUtil;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private double goalDegs = 0;
  private boolean isShooting = false;
  private double targetDegs = 0;

  private final double talonOffset;

  public Turret(TurretIO io) {
    this.io = io;
    io.updateInputs(inputs);
    double currentDegs = getPositionDegs();
    talonOffset = Units.degreesToRotations(currentDegs) * gearRatio - inputs.talonRotations;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    double currentDegs = getPositionDegs();
    inputs.truePositionDegs = currentDegs;
    inputs.positionDegs = (currentDegs - (2 * bufferDegs)) % 360;
    Logger.processInputs("Turret", inputs);
    GeneralUtil.logSubsystem(this, "Turret");

    if (isShooting) {
      targetDegs += bufferDegs * 2;
      if (Math.abs(targetDegs + 360 - currentDegs) < Math.abs(targetDegs - currentDegs)
          && (targetDegs + 360) <= maxAngleDegs) {
        targetDegs += 360;
      } else if (Math.abs(targetDegs - 360 - currentDegs) < Math.abs(targetDegs - currentDegs)
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

  public double getPositionDegs() {
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

  public boolean atGoal() {
    return Math.abs(inputs.positionDegs - goalDegs) < angleToleranceDegs
        && inputs.velocityDegsPerSec
            < velocityToleranceDegs; // TODO Change the velocity tolerance to be within a set
    // velocity, rather than just 0
  }

  public void setGoalDegs(double goalDegs) {
    this.goalDegs = goalDegs;
  }

  public void setIsShooting(boolean isShooting) {
    this.isShooting = isShooting;
  }
}
