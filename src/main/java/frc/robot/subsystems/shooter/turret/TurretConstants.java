package frc.robot.subsystems.shooter.turret;

import frc.robot.util.LoggedTunableNumber;

public class TurretConstants {
  // TODO please add "final" to all of these so they are actually constants
  public static int motorID = 5;
  public static int encoder1ID = 8;
  public static int encoder2ID = 9;

  public static boolean motorInverted = true;

  public static double turretGear = 85f;
  public static double encoder1Gear = 11f;
  public static double encoder2Gear = 10f;

  public static double gearRatio = 48.875; // turretGear / Motor Gear

  public static double trueMinAngleDegs = 0f;
  public static double trueMaxAngleDegs = 420f;

  public static double bufferDegs = (trueMaxAngleDegs - 360f) / 4f;
  public static double centerDegs = trueMaxAngleDegs / 2f;

  public static double minAngleDegs = trueMinAngleDegs + bufferDegs;
  public static double maxAngleDegs = trueMaxAngleDegs - bufferDegs;

  public static double minBufferAngleDegs = minAngleDegs + bufferDegs;
  public static double maxBufferAngleDegs = maxAngleDegs - bufferDegs;

  public static double totalGear = turretGear * (trueMaxAngleDegs / 360f);

  public static int totalRotations1 = (int) (totalGear / encoder1Gear);
  public static int totalRotations2 = (int) (totalGear / encoder2Gear);

  // TODO these also aren't used anywhere
  public static double rotationRatio1 = encoder1Gear / turretGear;
  public static double rotationRatio2 = encoder2Gear / turretGear;
  public static double extraDegs1 = 360f * (1 % totalRotations1);
  public static double extraDegs2 = 360f * (1 % totalRotations2);

  public static double angleToleranceDegs = 2.0; // todo change
  public static double velocityToleranceDegs = 0.5; // TODO change

  public static LoggedTunableNumber awayFromSetpointTolerance =
      new LoggedTunableNumber("Shooter/Turret/awayFromSetpointTolerance", 5.0);
}
