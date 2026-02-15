package frc.robot.subsystems.climb;

import static frc.robot.Constants.loopPeriodSecs;
import static frc.robot.subsystems.climb.ClimbConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ClimbIOSim implements ClimbIO {

  private final PIDController controller;
  private double appliedVolts = 0;
  private ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(1),
          gearRatio,
          elevatorMassKg,
          drumRadiusMeters,
          minHeightMeters,
          maxHeightMeters,
          true,
          0);

  public ClimbIOSim() {
    controller = new PIDController(kP.get(), 0.0, 0.0);
    sim.setState(0.0, 0.0);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    sim.update(loopPeriodSecs);
    inputs.appliedVolts = appliedVolts;
    inputs.positionMeters = sim.getPositionMeters();
  }

  @Override
  public void setPosition(double meters) {
    controller.setP(kP.get());
    appliedVolts = controller.calculate(sim.getPositionMeters(), meters) + kG.get();
    sim.setInputVoltage(appliedVolts);
  }
}
