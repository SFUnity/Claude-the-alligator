package.frc.robot.util;

public static class ShooterUtil {
    //Constants
    public static final double Gravity = 9.81; 
    public static final double ShooterMaxRPM = 0; // Placeholder value
    public static final double ShooterMinAngle = 45; 
    public static final double ShooterMaxAngle = 75; 

    //Calculations
    public static double calculateRequiredRPM(double horizontalDistanceMeters, double verticalDistanceMeters, double launchAngle) {
        // Convert angle to radians cuz java is stupid and doesn't use degrees (thank you google gemini)
        double angleRad = Math.toRadians(launchAngle);
        
        // Math shenanigans to calculate the required velocity or smth
        double requiredVelocity = Math.sqrt((Gravity * horizontalDistanceMeters * horizontalDistanceMeters) / (2 * Math.cos(angleRad) * (horizontalDistanceMeters * Math.tan(angleRad) - verticalDistanceMeters)));
        
        // Convert velocity to RPM or whatever measurement motors run on idk
        
        
        double calculationsNumerator = Gravity * horizontalDistanceMeters * horizontalDistanceMeters;
        double calculationsDenominator = 2 * Math.cos(angleRad) * (horizontalDistanceMeters * Math.tan(angleRad) - verticalDistanceMeters);
       
        if (requiredVelocitySquared < 0) {
            return null; 
        }
        double wheelRadius = 0; // Placeholder value
        double rpm = (requiredVelocity / (2 * Math.PI * wheelRadius)) * 60;

        return rpm;
    }

}