package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.junction.Logger;

public class RobotCommands {
  public static Command intake() {
    return Commands.run(() -> Logger.recordOutput("RobotCommands/Intake", true))
        .finallyDo((interrupted) -> Logger.recordOutput("RobotCommands/Intake", false));
  }

  public static Command climbExtend() {
    return Commands.run(() -> Logger.recordOutput("RobotCommands/ClimbExtend", true))
        .finallyDo((interrupted) -> Logger.recordOutput("RobotCommands/ClimbExtend", false));
  }

  public static Command climbRetract() {
    return Commands.run(() -> Logger.recordOutput("RobotCommands/ClimbRetract", true))
        .finallyDo((interrupted) -> Logger.recordOutput("RobotCommands/ClimbRetract", false));
  }

  public static Command shoot() {
    return Commands.run(() -> Logger.recordOutput("RobotCommands/Shoot", true))
        .finallyDo((interrupted) -> Logger.recordOutput("RobotCommands/Shoot", false));
  }

  public static Command sequence(Command... commands) {
    return new SequentialCommandGroup(commands);
  }
}
