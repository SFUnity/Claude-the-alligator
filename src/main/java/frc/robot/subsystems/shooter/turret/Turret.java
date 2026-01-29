package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private double goalDegs = 0;
  private boolean isShooting = false;
  private double targetDegs = 0;

  public Turret(TurretIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    inputs.positionDegs = getPositionDegs();
    Logger.processInputs("Turret", inputs);
    
    if (isShooting) {
      double currentDegs = getPositionDegs();
      targetDegs += bufferDegs * 2;
      if (Math.abs(targetDegs + 360 - currentDegs) < Math.abs(targetDegs - currentDegs)
          && (targetDegs + 360) <= maxAngleDegs) {
        targetDegs += 360;
      } else if (Math.abs(targetDegs - 360 - currentDegs) < Math.abs(targetDegs - currentDegs)
          && (targetDegs - 360) >= minAngleDegs) {
        targetDegs -= 360;
      }
      io.turnTurret(currentDegs, isShooting);
    } else {
      targetDegs += bufferDegs * 2;
      io.turnTurret(targetDegs, isShooting);
    }
  }

  public double getPositionDegs() {
    double truePosition = 0;
    double position1 = io.getEncoder1Degs();
    double position2 = io.getEncoder2Degs();

    position1 = Units.rotationsToDegrees(position1);
    position2 = Units.rotationsToDegrees(position2);

    // chinese remainder theorem from claude check later
    truePosition =
        ((encoder1gear ^ 2 * encoder2gear * 360) * position1
                + (encoder2gear ^ 2 * encoder1gear * 360) * position2)
            % (360 ^ 2 * encoder1gear * encoder2gear);

    return truePosition;
  }

  public boolean isAtGoal() {
    return Math.abs(inputs.positionDegs - goalDegs) < angleToleranceDegs;
  }

  public void setGoalDegs(double goalDegs) {
    this.goalDegs = goalDegs;
  }

  public void setIsShooting(boolean isShooting) {
    this.isShooting = isShooting;
  }
}
