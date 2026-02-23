package frc.robot.subsystems.rollers.kicker;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class KickerIOSim implements KickerIO {
  private double appliedVolts = 0.0;

  private static final DCMotor motorModel = DCMotor.getKrakenX60(1);
  private static final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, .025, 1), motorModel);

  private final LoggedTunableNumber dutyCyclekP =
      new LoggedTunableNumber("Kicker/simDutyCycleKp", 0);
  private final PIDController dutyCycleController =
      new PIDController(dutyCyclekP.get(), 0, 0, Constants.loopPeriodSecs);

  private final LoggedTunableNumber torquekP = new LoggedTunableNumber("Kicker/simTorqueKp", 0);
  private final PIDController torqueController =
      new PIDController(torquekP.get(), 0, 0, Constants.loopPeriodSecs);

  public KickerIOSim() {
    dutyCycleController.setSetpoint(10);
    torqueController.setSetpoint(10);
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> dutyCycleController.setP(dutyCyclekP.get()), dutyCyclekP);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> torqueController.setP(torquekP.get()), torquekP);

    // Update sim state
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(Constants.loopPeriodSecs);

    inputs.appliedVolts = appliedVolts;
    inputs.velocityRotsPerMin = sim.getAngularVelocityRPM();
    inputs.currentAmps = sim.getCurrentDrawAmps();
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
    double dutyCycleOutput = dutyCycleController.calculate(sim.getAngularVelocityRadPerSec());
    dutyCycleOutput = MathUtil.clamp(dutyCycleOutput, 0, 1.0);
    appliedVolts = motorModel.getVoltage(dutyCycleOutput, sim.getAngularVelocityRadPerSec());
  }

  @Override
  public void runTorqueControl() {
    double currentOutput = torqueController.calculate(sim.getAngularVelocityRadPerSec());
    currentOutput = MathUtil.clamp(currentOutput, 0, 40.0);
    appliedVolts = motorModel.getVoltage(currentOutput, sim.getAngularVelocityRadPerSec());
  }
}
