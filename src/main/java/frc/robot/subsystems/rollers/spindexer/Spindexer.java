package frc.robot.subsystems.rollers.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {

  private SpindexerIO io;

  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  public Command runVolts(){
    return Commands.none();
  }
  public Command stop(){
    return Commands.none();
  }
}
