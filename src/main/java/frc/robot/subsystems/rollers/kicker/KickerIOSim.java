package frc.robot.subsystems.rollers.kicker;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class KickerIOSim implements KickerIO {
  private double appliedVolts = 0.0;
  private double currentAmps = 0.0;
  private static final DCMotor motorModel = DCMotor.getKrakenX60(1);
  private static final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, .025, 1), motorModel);

  public KickerIOSim() {}

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = currentAmps;
    inputs.velocityRotsPerMin = sim.getAngularVelocityRPM();
  }

  @Override
  public void stop() {
    appliedVolts = 0;
  }

  @Override
  public void runVolts(double volts) {
    appliedVolts = volts;
  }

  @Override
  public void runDutyCycle() {
    appliedVolts = 10;
  }

  @Override
  public void runTorqueControl() {
    appliedVolts = 10;
  }
}
