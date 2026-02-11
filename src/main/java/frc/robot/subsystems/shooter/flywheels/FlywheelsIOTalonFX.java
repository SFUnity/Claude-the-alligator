package frc.robot.subsystems.shooter.flywheels;

import static frc.robot.subsystems.shooter.flywheels.FlywheelsConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class FlywheelsIOTalonFX implements FlywheelsIO {
  private final TalonFX leader = new TalonFX(leaderID);
  private final TalonFX follow = new TalonFX(followID);
  private final VoltageOut voltageOut =
      new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(Constants.loopPeriodSecs);
  private final MotionMagicVelocityVoltage motionMagicVelocity =
      new MotionMagicVelocityVoltage(0).withEnableFOC(true);

  public FlywheelsIOTalonFX() {
    var talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    var config = new TalonFXConfiguration();
    config.Slot0.kP = 999999.0;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
    config.MotorOutput.PeakForwardDutyCycle = 1.0;
    config.MotorOutput.PeakReverseDutyCycle = 0.0;

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
  public void run() {
    // duty-cycle bang bang
    leader.setControl(new VelocityDutyCycle(10.0).withEnableFOC(true));

    // Torque-current bang-bang
    leader.setControl(new VelocityTorqueCurrentFOC(10.0));
  }
}
