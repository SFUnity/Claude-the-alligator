package frc.robot.subsystems.shooter.flywheels;

import static frc.robot.subsystems.shooter.flywheels.FlywheelsConstants.*;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FlywheelsIOTalonFX implements FlywheelsIO {
  private final TalonFX leader = new TalonFX(leaderID);
  private final TalonFX follow = new TalonFX(followID);

  private final double updateFreqHz = 500;

  private final VoltageOut voltageOut =
      new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(updateFreqHz);
  private final VelocityDutyCycle dutyCycle =
      new VelocityDutyCycle(100).withUpdateFreqHz(updateFreqHz);
  private final VoltageOut slowDutyCycle = new VoltageOut(1);
  // private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withEnableFOC(true);

  private final VelocityTorqueCurrentFOC torqueCurrent =
      new VelocityTorqueCurrentFOC(100).withUpdateFreqHz(updateFreqHz);
  private final VelocityTorqueCurrentFOC torqueCurrentModifiable =
      new VelocityTorqueCurrentFOC(100).withUpdateFreqHz(updateFreqHz);

  public FlywheelsIOTalonFX() {
    var talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.Slot0.kP = 999999.0;
    talonFXConfigs.MotorOutput.PeakForwardDutyCycle = 1.0;
    talonFXConfigs.MotorOutput.PeakReverseDutyCycle = 0.0;

    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 80.0;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 60.0;

    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfigs.Feedback.SensorToMechanismRatio = gearRatio;

    tryUntilOk(5, () -> leader.getConfigurator().apply(talonFXConfigs, 0.25));
    tryUntilOk(5, () -> follow.getConfigurator().apply(talonFXConfigs, 0.25));

    tryUntilOk(
        5,
        () ->
            follow.setControl(
                new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed)
                    .withUpdateFreqHz(updateFreqHz)));
  }

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    inputs.appliedVolts = leader.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrent = leader.getSupplyCurrent().getValueAsDouble();
    inputs.statorCurrent = leader.getStatorCurrent().getValueAsDouble();
    inputs.velocityRotsPerMin = leader.getVelocity().getValueAsDouble() * 60;
  }

  @Override
  public void stop() {
    leader.setControl(voltageOut.withOutput(0));
  }

  @Override
  public void runDutyCycle() {
    leader.setControl(dutyCycle.withEnableFOC(true));
  }

  @Override
  public void runSlowDutyCycle() {
    leader.setControl(slowDutyCycle.withEnableFOC(true));
  }

  @Override
  public void runTorqueControl() {
    leader.setControl(torqueCurrent);
  }

  @Override
  public void runTorqueControl(double rps) {
    leader.setControl(torqueCurrentModifiable.withVelocity(rps));
  }

  @Override
  public void runVolts(double volts) {
    leader.setControl(voltageOut.withOutput(volts));
  }
}
