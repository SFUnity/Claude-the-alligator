package frc.robot.subsystems.shooter.flywheels;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FlywheelsIOSim implements FlywheelsIO {
  private double appliedVolts = 0.0;

  private static final DCMotor motorModel = DCMotor.getKrakenX60(1);
  private static final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, .025, 1), motorModel);

  public FlywheelsIOSim() {}

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    // Update sim state
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(Constants.loopPeriodSecs);

    inputs.appliedVolts = appliedVolts;
    inputs.velocityRotsPerMin = sim.getAngularVelocityRPM();
    inputs.statorCurrent = sim.getCurrentDrawAmps();
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
