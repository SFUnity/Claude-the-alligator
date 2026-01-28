package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.GenericRollerIO.GenericRollerIOInputs;

public class GenericRoller extends SubsystemBase {
  private final GenericRollerIO io;
  protected final GenericRollerIOInputs inputs;

  public enum GoalStates {
    STOP(0);

    private double volts;

    private GoalStates(double volts) {
      this.volts = volts;
    }

    public double getVolts() {
      return volts;
    }
  }

  private GoalStates goalState;

  public GenericRoller(GenericRollerIO io) {
    this.io = io;
    inputs = new GenericRollerIOInputs();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.runVolts(goalState.getVolts());
  }

  public void setState(GoalStates goalState) {
    this.goalState = goalState;
  }
}
