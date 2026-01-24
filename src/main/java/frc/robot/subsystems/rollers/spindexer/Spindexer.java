package frc.robot.subsystems.rollers.spindexer;

import frc.robot.subsystems.rollers.GenericRoller;
import frc.robot.subsystems.rollers.GenericRoller.VoltageGoal;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class Spindexer extends GenericRoller<Spindexer.Goal> {

  public enum Goal implements VoltageGoal {
    IDLING(() -> 0.0),
    FLOOR_INTAKING(new LoggedTunableNumber("Indexer/FloorIntakingVoltage", 6.0)),
    STATION_INTAKING(new LoggedTunableNumber("Indexer/StationIntakingVoltage", -6.0)),
    SHOOTING(new LoggedTunableNumber("Indexer/ShootingVoltage", 12.0)),
    EJECTING(new LoggedTunableNumber("Indexer/EjectingVoltage", -8.0));

    private final DoubleSupplier voltageSupplier;

    private Goal(DoubleSupplier voltageSupplier) {
      this.voltageSupplier = voltageSupplier;
    }

    public DoubleSupplier getVoltageSupplier() {
      return voltageSupplier;
    }
  }

  public Spindexer.Goal getGoal() {
    return goal;
  }

  private Spindexer.Goal goal = Spindexer.Goal.IDLING;

  public Spindexer(SpindexerIO io) {
    super("Indexer", io);
  }
}
