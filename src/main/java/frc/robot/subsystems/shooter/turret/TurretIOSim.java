package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class TurretIOSim implements TurretIO {
  private final DCMotor gearbox = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.001, 1.0), gearbox);

  private double currentOutput = 0.0;
  private double appliedVoltage = 0.0;
  private boolean currentControl = false;

  public TurretIOSim() {}

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    sim.setInputVoltage(currentOutput);
    sim.update(Constants.loopPeriodSecs);

    inputs.appliedVolts = currentOutput;
    inputs.talonRotations = sim.getAngularPositionRotations();
    inputs.velocityDegsPerSec =
        Units.radiansToDegrees(
            Units.rotationsPerMinuteToRadiansPerSecond(sim.getAngularVelocityRPM()));
  }

  @Override
  public void turnTurret(double targetRotations, double targetVelocity, double kP, double kD) {
    currentOutput =
        (Units.rotationsToRadians(targetRotations) - sim.getAngularPositionRad()) * kP
            + (Units.degreesToRadians(targetVelocity) - sim.getAngularVelocityRadPerSec()) * kD;
  }

  @Override
  public void stop() {
    currentOutput = 0;
  }
}
