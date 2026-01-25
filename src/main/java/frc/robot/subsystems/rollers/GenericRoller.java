package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public abstract class GenericRoller<G extends GenericRoller.VoltageGoal> extends SubsystemBase{
  public interface VoltageGoal {
    DoubleSupplier getVoltageSupplier();
  }

  public abstract G getGoal();

  private final String name;
  private final GenericRollerIO io;
  protected final GenericRollerIOInputsAutoLogged inputs = new GenericRollerIOInputsAutoLogged();
  protected final Timer stateTimer = new Timer();
  private G lastGoal;

  public GenericRoller(String name, GenericRollerIO io) {
    this.name = name;
    this.io = io;

    stateTimer.start();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    if (getGoal() != lastGoal) {
      stateTimer.reset();
      lastGoal = getGoal();
    }

    io.runVolts(getGoal().getVoltageSupplier().getAsDouble());
    Logger.recordOutput("Rollers/" + name + "Goal", getGoal().toString());
  }

  public abstract void setGoal(G goal);
}
