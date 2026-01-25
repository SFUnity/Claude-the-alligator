package frc.robot.subsystems.rollers.spindexer;

import frc.robot.subsystems.rollers.GenericRollerIOTalonFX;

public class SpindexerIOTalonFX extends GenericRollerIOTalonFX implements SpindexerIO {

    SpindexerIOTalonFX(int id, int currentLimitAmps, boolean invert) {
        super(id, currentLimitAmps, invert);
    }
}
