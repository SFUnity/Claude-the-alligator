package frc.robot.subsystems.shooter.flywheels;

import static frc.robot.subsystems.shooter.flywheels.FlywheelsConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
// TODO remove unused
import frc.robot.subsystems.shooter.flywheels.FlywheelsIO.FlywheelsIOInputs;

public class FlywheelsIOSim implements FlywheelsIO {
  private static final DCMotor motorModel = DCMotor.getKrakenX60(1);
  private static final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, .025, 1), motorModel);
  private PIDController pid = new PIDController(kP.get(), 0, kD.get());
  private double appliedVolts = 0;

  public FlywheelsIOSim() {}

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(Constants.loopPeriodSecs);

    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrent = sim.getCurrentDrawAmps();
    // TODO why seconds?!?!?!?!?
    inputs.velocityRotsPerSec = sim.getAngularVelocityRPM() * 60;
  }

  @Override
  public void runVelocity(double rps) {
    appliedVolts = pid.calculate(rps);
  }

  @Override
  public void ready() {
    appliedVolts = readyVolts.get();
  }
}
