package frc.robot.subsystems.intakePivot;

import static frc.robot.subsystems.intakePivot.IntakePivotConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class IntakePivotIOTalon implements IntakePivotIO {
  private final SparkMax pivot = new SparkMax(pivotID, MotorType.kBrushless);
  private final SparkMax rollers = new SparkMax(rollersID, MotorType.kBrushless);
  private final RelativeEncoder encoder = pivot.getEncoder();
  private final SparkClosedLoopController pid = pivot.getClosedLoopController();

  public IntakePivotIOTalon() {}

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.pivotCurrentPositionDeg = encoder.getPosition();
    inputs.pivotAppliedVolts = pivot.getAppliedOutput() * pivot.getBusVoltage();
    inputs.pivotCurrentAmps = pivot.getOutputCurrent();

    inputs.rollersAppliedVolts = rollers.getAppliedOutput() * rollers.getBusVoltage();
    inputs.rollersCurrentAmps = rollers.getOutputCurrent();
  }

  @Override
  public void runRollers(double volts) {
    rollers.setVoltage(volts);
  }

  @Override
  public void runPivot(double volts) {
    pivot.setVoltage(volts);
  }

  @Override
  public void setPivotPosition(double setpointDeg) {
    pid.setReference(setpointDeg * pivotPositionFactor, ControlType.kPosition);
  }

  @Override
  public void resetEncoder(double position) {
    encoder.setPosition(position);
  }
}
