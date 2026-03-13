package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOSim implements HoodIO {
  private double angle;
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          50,
          0.5,
          armLengthMeters,
          minAngleRads,
          maxAngleRads,
          false,
          minAngleRads);
  private final PIDController controller;

  public HoodIOSim() {
    controller = new PIDController(330, 0.0, 0.0);
    sim.setState(minAngleRads, 0.0);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.positionDeg = angle;
  }

  @Override
  public void setPosition(double positionDeg) {
    angle = positionDeg;
  }
}
