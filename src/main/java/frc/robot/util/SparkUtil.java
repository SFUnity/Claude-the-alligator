// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.util;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SparkUtil {
  /** Stores whether any error was has been detected by other utility methods. */
  public static boolean sparkStickyFault = false;

  /** Processes a value from a Spark only if the value is valid. */
  public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
    double value = supplier.getAsDouble();
    if (spark.getLastError() == REVLibError.kOk) {
      consumer.accept(value);
    } else {
      sparkStickyFault = true;
    }
  }

  /** Processes a value from a Spark only if the value is valid. */
  public static void ifOk(
      SparkBase spark, DoubleSupplier[] suppliers, Consumer<double[]> consumer) {
    double[] values = new double[suppliers.length];
    for (int i = 0; i < suppliers.length; i++) {
      values[i] = suppliers[i].getAsDouble();
      if (spark.getLastError() != REVLibError.kOk) {
        sparkStickyFault = true;
        return;
      }
    }
    consumer.accept(values);
  }

  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(SparkBase spark, Supplier<REVLibError> command) {
    int maxAttempts = 5;

    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error == REVLibError.kOk) {
        // System.out.println("Configured spark: " + spark.getDeviceId());
        break;
      } else {
        sparkStickyFault = true;
      }
    }
  }

  /**
   *
   *
   * <h2>persistParameters SHOULD ONLY BE TRUE AT STARTUP</h2>
   *
   * <p>Configuring a SPARK MAX and SPARK Flex differs from other devices in REVLib with the
   * addition of the persistMode parameter in their configure() methods, which specifies whether the
   * configuration settings applied to the device should be persisted between power cycles.
   *
   * <p>Persisting parameters involves saving them to the SPARK controller's memory, which is
   * time-intensive and blocks communication with the device. To provide flexibility, this process
   * is not automatic, as this behavior may be unnecessary or undesirable in some cases. Therefore,
   * users must manually specify the persist mode, and to help avoid possible pitfalls, it is a
   * required parameter.
   */
  public static void configureSpark(
      SparkBase spark, SparkMaxConfig config, boolean persistParameters) {
    tryUntilOk(
        spark,
        () ->
            spark.configure(
                config,
                persistParameters
                    ? ResetMode.kResetSafeParameters
                    : ResetMode.kNoResetSafeParameters,
                persistParameters
                    ? PersistMode.kPersistParameters
                    : PersistMode.kNoPersistParameters));
  }

  public static void logSparkMax(String key, SparkMax sparkMax) {
    Logger.recordOutput(key + "/inverted", sparkMax.configAccessor.getInverted());
    Logger.recordOutput(key + "/currentLimit", sparkMax.configAccessor.getSmartCurrentLimit());
    Logger.recordOutput(key + "/idleMode", sparkMax.configAccessor.getIdleMode());
  }

  public static SparkMaxConfig sparkConfig(boolean inverted, double encoderToMechanismRatio) {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(inverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60)
        .voltageCompensation(12.0);
    config
        .encoder
        .positionConversionFactor(encoderToMechanismRatio)
        .velocityConversionFactor(encoderToMechanismRatio)
        .uvwAverageDepth(2);
    config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    return config;
  }
}
