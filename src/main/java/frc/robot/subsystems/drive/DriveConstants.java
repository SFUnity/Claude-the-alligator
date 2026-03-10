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

package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = Units.feetToMeters(10.5);
  public static final double trackWidth = Units.inchesToMeters(20.75);
  public static final double wheelBase = trackWidth;
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final double maxAngularSpeedRadiansPerSec = maxSpeedMetersPerSec / driveBaseRadius;
  public static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(Drive.getModuleTranslations());
  /** Frame perimeter to outer edge of bumper on 1 side. Based on CAD */
  public static final double bumperWidth = Units.inchesToMeters(3.25);
  /** Floor to top of bumper. Based on CAD */
  public static final double bumperHeight = Units.inchesToMeters(5.75);
  public static final double lengthInches = 27.5;
  public static final double widthInches = 26.5;

  // Drive PID configuration
  public static final LoggedTunableNumber driveKp;
  public static final double driveKs;
  public static final double driveKv;

  // Time between Vision reading and gyro update in seconds
  public static final double gyroVisionLatencyLimit =
      0.05; // TODO tune to make sure it wont fuck up gyro but will actually let readings in

  static {
    switch (Constants.currentMode) {
      default:
        driveKp = new LoggedTunableNumber("Drive/ModuleTunables/driveKp", 0);
        driveKs = 0.24441;
        driveKv = 0.67991;
        break;
      case SIM:
        driveKp = new LoggedTunableNumber("Drive/SimModuleTunables/driveKp", 0.29);
        driveKs = 0.0;
        driveKv = 0.0;
        break;
    }
  }

  // Turn PID configuration
  public static final LoggedTunableNumber turnKp;
  public static final LoggedTunableNumber turnKd;
  public static final LoggedTunableNumber turnKs;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  static {
    switch (Constants.currentMode) {
      default:
        turnKp = new LoggedTunableNumber("Drive/ModuleTunables/turnKp", 120);
        turnKd = new LoggedTunableNumber("Drive/ModuleTunables/turnKd", .25);
        turnKs = new LoggedTunableNumber("Drive/ModuleTunables/turnKs", 0);
        break;
      case SIM:
        turnKp = new LoggedTunableNumber("Drive/SimModuleTunables/turnKp", 14.0);
        turnKd = new LoggedTunableNumber("Drive/SimModuleTunables/turnKd", 0.0);
        turnKs = new LoggedTunableNumber("Drive/SimModuleTunables/turnKs", 0.0);
        break;
    }
  }
}
