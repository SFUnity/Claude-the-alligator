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

    public static double bufferDegs = (trueMaxAngleDegs - 360)/4;
    public static double centerDegs = trueMaxAngleDegs/2;

    public static double minAngleDegs = trueMinAngleDegs + bufferDegs;
    public static double maxAngleDegs = trueMaxAngleDegs - bufferDegs;
}
