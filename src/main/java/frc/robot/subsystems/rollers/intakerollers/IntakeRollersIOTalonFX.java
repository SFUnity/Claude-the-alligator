package frc.robot.subsystems.rollers.intakerollers;

import static frc.robot.Constants.loopPeriodSecs;
import static frc.robot.subsystems.rollers.intakerollers.IntakeRollersConstants.*;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeRollersIOTalonFX implements IntakeRollersIO {
  private final TalonFX rollerMotor = new TalonFX(intakeRollersMotorID);

  private final VoltageOut voltageOut =
      new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(loopPeriodSecs);

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {
    inputs.appliedVolts = rollerMotor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = rollerMotor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    rollerMotor.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void stop() {
    rollerMotor.setControl(voltageOut.withOutput(0));
  }
}
