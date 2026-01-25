package frc.robot.subsystems.rollers.kicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.rollers.kicker.KickerConstants.*;
import org.littletonrobotics.junction.Logger;


public class Kicker extends SubsystemBase {
    private final KickerIO io;
    private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

    public Kicker(KickerIO io){
        this.io = io;
    }
    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Kicker",inputs);

    }

    public Command runVolts(){
        return run(() -> io.runVolts(kickerSpeedVolts.get()));
    }

    public Command stop(){
        return run(()-> io.runVolts(0));
    }
    
  
}
