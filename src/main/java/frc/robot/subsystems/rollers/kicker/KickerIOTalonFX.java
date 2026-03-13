package frc.robot.subsystems.rollers.kicker;

import static frc.robot.Constants.loopPeriodSecs;
import static frc.robot.subsystems.rollers.kicker.KickerConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class KickerIOTalonFX implements KickerIO {
  private final Alert laserCanInvalidAlert =
      new Alert("Laser can Target out of range", AlertType.kError);
  private LaserCan lc;
  private final TalonFX rollerMotor = new TalonFX(kickerMotorID);

  private final VoltageOut voltageOut =
      new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(loopPeriodSecs);
  // private final DutyCycleOut voltageOut = new DutyCycleOut(1);
  private final VoltageOut slowVoltageOut = new VoltageOut(1);
  // private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withEnableFOC(true);

  private final VelocityTorqueCurrentFOC torqueCurrent = new VelocityTorqueCurrentFOC(750);
  private final VelocityTorqueCurrentFOC torqueCurrentModifiable =
      new VelocityTorqueCurrentFOC(100);

  @SuppressWarnings("resource")
  public KickerIOTalonFX() {
    lc = new LaserCan(laserCANID);

    try {
      lc.setRangingMode(LaserCan.RangingMode.SHORT);
      lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      new Alert("Configuration failed" + e, AlertType.kError).set(true);
    }
    ;

    var talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 80.0;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 60.0;
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfigs.Slot0.kP = 999999.0;
    talonFXConfigs.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    talonFXConfigs.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
    talonFXConfigs.MotorOutput.PeakForwardDutyCycle = 1.0;
    talonFXConfigs.MotorOutput.PeakReverseDutyCycle = 0.0;

    tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(talonFXConfigs, 0.25));
  }

  @SuppressWarnings("resource")
  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.appliedVolts = rollerMotor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = rollerMotor.getSupplyCurrent().getValueAsDouble();
    inputs.velocityRotsPerMin = rollerMotor.getVelocity().getValueAsDouble() * 60;

    LaserCan.Measurement measurement = lc.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      inputs.laserMeasurementInches = Units.metersToInches(measurement.distance_mm * 1000);
      laserCanInvalidAlert.set(false);
    } else {
      laserCanInvalidAlert.set(true);
    }
  }

  @Override
  public void stop() {
    rollerMotor.setControl(voltageOut.withOutput(0));
  }

  @Override
  public void runDutyCycle() {
    rollerMotor.setControl(voltageOut.withOutput(10).withEnableFOC(true));
  }

  @Override
  public void runSlowDutyCycle(double rpm) {
    rollerMotor.setControl(slowVoltageOut.withOutput(rpm / 6000 * 10).withEnableFOC(true));
  }

  // @Override
  // public void runTorqueControl() {
  //   rollerMotor.setControl(torqueCurrent);
  // }

  // @Override
  // public void runTorqueControl(double rps) {
  //   rollerMotor.setControl(torqueCurrentModifiable.withVelocity(rps));
  // }

  @Override
  public void runVolts(double volts) {
    rollerMotor.setControl(voltageOut.withOutput(volts));
  }
}
