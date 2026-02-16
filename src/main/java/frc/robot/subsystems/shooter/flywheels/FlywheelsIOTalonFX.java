package frc.robot.subsystems.shooter.flywheels;

import static frc.robot.subsystems.shooter.flywheels.FlywheelsConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FlywheelsIOTalonFX implements FlywheelsIO {
  private final TalonFX leader = new TalonFX(leaderID);
  private final TalonFX follow = new TalonFX(followID);
  private final VelocityDutyCycle dutyCycle = new VelocityDutyCycle(10.0).withEnableFOC(true);
  private final VelocityTorqueCurrentFOC torqueControl = new VelocityTorqueCurrentFOC(10.0);

  public FlywheelsIOTalonFX() {
    var talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.Slot0.kP = 0.0;

    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 80.0;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 60.0;

    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    leader.getConfigurator().apply(talonFXConfigs);

    follow.getConfigurator().apply(talonFXConfigs);
    follow.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    inputs.appliedVolts = leader.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrent = leader.getSupplyCurrent().getValueAsDouble();
    inputs.statorCurrent = leader.getStatorCurrent().getValueAsDouble();
    inputs.velocityRotsPerMin = leader.getVelocity().getValueAsDouble() * 60;
  }

  @Override
  public void runDutyCycle() {
    // duty-cycle bang bang
    leader.setControl(dutyCycle);
  }

  @Override
  public void runTorqueControl() {
    // Torque-current bang-bang
    leader.setControl(torqueControl);
  }
}
