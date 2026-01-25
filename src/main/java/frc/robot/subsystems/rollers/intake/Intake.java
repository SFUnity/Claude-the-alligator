package frc.robot.subsystems.rollers.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.rollers.intake.IntakeConstants.*;

public class Intake extends SubsystemBase{
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("intake",inputs);
    }

    public Command runVolts(){
        return run(()-> io.runVolts(intakeSpeedVolts.get()));
    }

    public Command stop(){
        return run(()-> io.runVolts(0));
    }
}
