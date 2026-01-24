package frc.robot.subsystems.shooter.flywheels;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.flywheels.FlywheelsIO.FlywheelsIOInputs;

public class FlywheelsIOSim implements FlywheelsIO {
  private static final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(0, 0), null, null);
  private PIDController pid = new PIDController(0, 0, 0);
  private double appliedVolts = 0;

  public FlywheelsIOSim() {}

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(Constants.loopPeriodSecs);

    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.velocityRotsPerSec = sim.getAngularVelocityRPM() * 60;
  }

  @Override
  public void runVelocity(double velocity) {
    appliedVolts = pid.calculate(velocity);
  }

  @Override
  public void idle() {
    appliedVolts = 0;
  }
}
