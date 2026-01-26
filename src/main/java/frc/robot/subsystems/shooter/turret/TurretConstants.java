package frc.robot.subsystems.shooter.turret;

public class TurretConstants {
  public static int motorID = 0;
  public static int encoder1ID = 0;
  public static int encoder2ID = 0;

  public static boolean motorInverted = false;

  public static int turretGear = 85;
  public static int encoder1gear = 11;
  public static int encoder2gear = 10;

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

  public static double totalRotations1 = totalGear / encoder1gear;
  public static double totalRotations2 = totalGear / encoder2gear;

  public static double rotationRatio1 = encoder1gear / turretGear;
  public static double rotationRatio2 = encoder2gear / turretGear;

  public static double extraDegs1 = 360 * (1 % totalRotations1);
  public static double extraDegs2 = 360 * (1 % totalRotations2);

  public static double shootKA = 0.005;
  public static double shootKV = 0.1;

  public static double noshootKA = 0.1;
  public static double noshootKV = 0.2;
}
