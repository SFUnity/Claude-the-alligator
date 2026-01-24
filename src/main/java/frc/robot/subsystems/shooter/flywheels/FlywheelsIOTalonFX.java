package frc.robot.subsystems.shooter.flywheels;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class FlywheelsIOTalonFX implements FlywheelsIO {
  private final TalonFX leader = new TalonFX(0);
  private final TalonFX follow = new TalonFX(0);
  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(0);
  private final MotionMagicVelocityVoltage motionMagicVelocity =
      new MotionMagicVelocityVoltage(0).withEnableFOC(true);

  public FlywheelsIOTalonFX() {
    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0;
    slot0Configs.kV = 0;
    slot0Configs.kA = 0.0;
    slot0Configs.kP = 0;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    leader.getConfigurator().apply(talonFXConfigs);
    follow.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    inputs.appliedVolts = leader.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = leader.getSupplyCurrent().getValueAsDouble();
    inputs.velocityRotsPerSec = leader.getVelocity().getValueAsDouble();
  }

  @Override
  public void runVelocity(double velocity) {
    leader.setControl(motionMagicVelocity.withVelocity(velocity));
  }

  @Override
  public void idle() {
    leader.setControl(voltageOut.withOutput(0.0));
  }
}
