package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotCommands {
    public static Command intake() {
        return Commands.run(() -> Logger.recordOutput("RobotCommands/Intake", true))
                .finallyDo((interrupted) -> Logger.recordOutput("RobotCommands/Intake", false));
    }

    public static Command climb() {
        return Commands.run(() -> Logger.recordOutput("RobotCommands/Climb", true))
                .finallyDo((interrupted) -> Logger.recordOutput("RobotCommands/Climb", false));
    }
    public static Command score() {
        return Commands.run(() -> Logger.recordOutput("RobotCommands/Score", true))
                .finallyDo((interrupted) -> Logger.recordOutput("RobotCommands/Score", false));
    }
}
