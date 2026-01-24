package frc.robot.subsystems.shooter.flywheels;

import frc.robot.Constants;
import static frc.robot.subsystems.shooter.flywheels.FlywheelsConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class FlywheelsIOTalonFX implements FlywheelsIO {
  private final TalonFX leader = new TalonFX(leaderID);
  private final TalonFX follow = new TalonFX(followID);
  private final VoltageOut voltageOut =
      new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(Constants.loopPeriodSecs);
  private final MotionMagicVelocityVoltage motionMagicVelocity =
      new MotionMagicVelocityVoltage(0).withEnableFOC(true);

  public FlywheelsIOTalonFX() {
    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0;
    slot0Configs.kV = kV;
    slot0Configs.kA = kA;
    slot0Configs.kP = kP;
    slot0Configs.kI = 0;
    slot0Configs.kD = kD;
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 80.0;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 60.0;

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
        motionMagicCruiseVelocity; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        motionMagicAcceleration; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk =
        motionMagicJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)
    
    leader.getConfigurator().apply(talonFXConfigs);
    follow.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    inputs.appliedVolts = leader.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrent = leader.getSupplyCurrent().getValueAsDouble();
    inputs.statorCurrent = leader.getStatorCurrent().getValueAsDouble();
    inputs.velocityRotsPerSec = leader.getVelocity().getValueAsDouble();
  }

  @Override
  public void runVelocity(double velocity) {
    leader.setControl(motionMagicVelocity.withVelocity(velocity));
  }

  @Override
  public void idle() {
    leader.setControl(voltageOut.withOutput(idleVolts));
  }
}
