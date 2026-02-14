package frc.robot.subsystems.rollers.kicker;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.loopPeriodSecs;
import static frc.robot.subsystems.rollers.kicker.KickerConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class KickerIOTalonFX implements KickerIO {
  private final TalonFX rollerMotor = new TalonFX(kickerMotorID);
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withEnableFOC(true);

  private final VoltageOut voltageOut =
      new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(loopPeriodSecs);
  private final VelocityDutyCycle dutyCycle = new VelocityDutyCycle(10);
  private final VelocityTorqueCurrentFOC torqueCurrent = new VelocityTorqueCurrentFOC(10);

  public KickerIOTalonFX() {
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;

    // slot0Configs.kS = 0;
    // slot0Configs.kV = kV.get();
    // slot0Configs.kA = kA.get();
    // slot0Configs.kP = kP.get();
    // slot0Configs.kI = 0;
    // slot0Configs.kD = kD.get();
    // talonFXConfigs.CurrentLimits.StatorCurrentLimit = 80.0;
    // talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    // talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 60.0;

    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    talonFXConfigs.Slot0.kP = 999999.0;
    talonFXConfigs.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    talonFXConfigs.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
    talonFXConfigs.MotorOutput.PeakForwardDutyCycle = 1.0;
    talonFXConfigs.MotorOutput.PeakReverseDutyCycle = 0.0;
    tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(talonFXConfigs, 0.25));
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.appliedVolts = rollerMotor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = rollerMotor.getSupplyCurrent().getValueAsDouble();
    inputs.velocityRotsPerMin = rollerMotor.getVelocity().getValueAsDouble()*60;
  }

  @Override
  public void stop() {
    rollerMotor.setControl(voltageOut.withOutput(0));
  }

  // @Override
  // public void runVelocity(double rps) {
  //   rollerMotor.setControl(velocityVoltage.withVelocity((rps / 60)));
  // }
  @Override
  public void runDutyCycle() {
    rollerMotor.setControl(dutyCycle.withEnableFOC(true));
  }

  @Override
  public void runTorqueControl(){
    rollerMotor.setControl(torqueCurrent);
  }

  @Override
  public void runVolts(double volts){
    rollerMotor.setControl(voltageOut.withOutput(volts));
  }
}
