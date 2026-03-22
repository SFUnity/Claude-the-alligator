package frc.robot.subsystems.shooter.turret;

import frc.robot.util.LoggedTunableNumber;

public class TurretConstants {
  public static final int motorID = 5;
  public static final int encoder1ID = 8;
  public static final int encoder2ID = 9;

  public static final boolean motorInverted = true;

  public static final double turretGear = 85f;
  public static final double encoder1Gear = 11f;
  public static final double encoder2Gear = 10f;

  public static final double gearRatio = 48.875; // turretGear / Motor Gear

  public static final double trueMinAngleDegs = 0f;
  public static final double trueMaxAngleDegs = 380f;

  public static final double bufferDegs = (trueMaxAngleDegs - 360f) / 4f;
  public static final double centerDegs = trueMaxAngleDegs / 2f;

  public static final double minAngleDegs = trueMinAngleDegs + bufferDegs;
  public static final double maxAngleDegs = trueMaxAngleDegs - bufferDegs;

  public static final double minBufferAngleDegs = minAngleDegs + bufferDegs;
  public static final double maxBufferAngleDegs = maxAngleDegs - bufferDegs;

  public static final double totalGear = turretGear * (trueMaxAngleDegs / 360f);

  public static final int totalRotations1 = (int) (totalGear / encoder1Gear);
  public static final int totalRotations2 = (int) (totalGear / encoder2Gear);

  // TODO these also aren't used anywhere
  public static final double rotationRatio1 = encoder1Gear / turretGear;
  public static final double rotationRatio2 = encoder2Gear / turretGear;
  public static final double extraDegs1 = 360f * (1 % totalRotations1);
  public static final double extraDegs2 = 360f * (1 % totalRotations2);

  public static final double angleToleranceDegs = 2.0; // todo change
  public static final double velocityToleranceDegs = 0.5; // TODO change

  public static final double turretOffsetDegs = 0;

  public static final LoggedTunableNumber awayFromSetpointTolerance =
      new LoggedTunableNumber("Shooter/Turret/awayFromSetpointTolerance", 5.0);
}
