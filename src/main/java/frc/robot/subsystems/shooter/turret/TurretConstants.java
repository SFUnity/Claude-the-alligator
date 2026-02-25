package frc.robot.subsystems.shooter.turret;

import frc.robot.util.LoggedTunableNumber;

public class TurretConstants {
  // TODO please add "final" to all of these so they are actually constants
  public static int motorID = 5;
  public static int encoder1ID = 8;
  public static int encoder2ID = 9;

  public static boolean motorInverted = true;

  public static int turretGear = 85;
  public static int encoder1Gear = 11;
  public static int encoder2Gear = 10;

  public static double gearRatio = 48.875; // turretGear / Motor Gear

  public static double trueMinAngleDegs = 0;
  public static double trueMaxAngleDegs = 420;

  public static double bufferDegs = (trueMaxAngleDegs - 360) / 4;
  public static double centerDegs = trueMaxAngleDegs / 2;

  public static double minAngleDegs = trueMinAngleDegs + bufferDegs;
  public static double maxAngleDegs = trueMaxAngleDegs - bufferDegs;

  public static double minBufferAngleDegs = minAngleDegs + bufferDegs;
  public static double maxBufferAngleDegs = maxAngleDegs - bufferDegs;

  public static double totalGear = turretGear * (trueMaxAngleDegs / 360);

  public static double totalRotations1 = totalGear / encoder1Gear;
  public static double totalRotations2 = totalGear / encoder2Gear;

  // TODO these also aren't used anywhere
  public static double rotationRatio1 = encoder1Gear / turretGear;
  public static double rotationRatio2 = encoder2Gear / turretGear;
  public static double extraDegs1 = 360 * (1 % totalRotations1);
  public static double extraDegs2 = 360 * (1 % totalRotations2);

  public static double angleToleranceDegs = 2.0; // todo change
  public static double velocityToleranceDegs = 0.5; // TODO change

  public static LoggedTunableNumber awayFromSetpointTolerance =
      new LoggedTunableNumber("Shooter/Turret/awayFromSetpointTolerance", 5.0);
}
