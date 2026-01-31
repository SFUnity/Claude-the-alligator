package frc.robot.util;

public class ShooterUtil implements AutoCloseable {
    //Constants
    public static final double Gravity = 9.81; 
    public static final double ShooterMaxRPM = 3500; 
    public static final double ShooterMinAngle = 45; 
    public static final double ShooterMaxAngle = 75; 

    //returning three values so we need a custom helper class
    public static class ShooterSolution {
        public final double angleDeg;
        public final double rpm;
        public final double TurnAngleDeg;

        public ShooterSolution(double angleDeg, double rpm, double TurnAngleDeg) {
            this.angleDeg = angleDeg;
            this.rpm = rpm;
            this.TurnAngleDeg = TurnAngleDeg;
        }
    }

    //Calculations
    public static ShooterSolution calculateOptimalShot(double horizontalDistanceMeters, double verticalDistanceMeters, double launchAngle) {
        
        //I'll just assume minimum angle is always optimal for now 
        double optimalAngleRad = Math.atan(
            (verticalDistanceMeters +
             Math.sqrt(
                 horizontalDistanceMeters * horizontalDistanceMeters +
                 verticalDistanceMeters * verticalDistanceMeters
             )) / horizontalDistanceMeters
        );
        //Added a bit of tolerance by increasing the angle by 2 degrees
        double optimalAngleDeg = Math.toDegrees(optimalAngleRad) + 2;
        double optimalAngle = Math.toRadians(optimalAngleDeg);

        // Math shenanigans to calculate the required velocity or smth
        double requiredVelocity = Math.sqrt((Gravity * horizontalDistanceMeters * horizontalDistanceMeters) / (2 * Math.cos(optimalAngle) * (horizontalDistanceMeters * Math.tan(optimalAngle) - verticalDistanceMeters)));
        
        
        double calculationsNumerator = Gravity * horizontalDistanceMeters * horizontalDistanceMeters;
        double calculationsDenominator = 2 * Math.cos(optimalAngle) * (horizontalDistanceMeters * Math.tan(optimalAngle) - verticalDistanceMeters);

        if (calculationsNumerator / calculationsDenominator < 0) {
            return null;
        }
        
        
        // Convert velocity to RPM or whatever measurement motors run on idk
        double wheelRadius = 0.0508;
        double rpm = (requiredVelocity / (2 * Math.PI * wheelRadius)) * 60;

        //correcting angle instead if required rpm is too large
        if (rpm > ShooterMaxRPM) {
            // Solve quadratic for tan(theta) using v_max and basically reverse the projectile motion equations
            double v_max = 2 * Math.PI * wheelRadius * (ShooterMaxRPM / 60.0);
            double a = Gravity * horizontalDistanceMeters * horizontalDistanceMeters / (2 * v_max * v_max);
            double b = -horizontalDistanceMeters;
            double c = a + verticalDistanceMeters;

            double discriminant = b*b - 4*a*c;

            if (discriminant >= 0) {
                double T1 = (-b + Math.sqrt(discriminant)) / (2*a);
                double T2 = (-b - Math.sqrt(discriminant)) / (2*a);

                double newAngleRad = Math.atan(Math.min(T1, T2));
                double newAngleDeg = Math.toDegrees(newAngleRad);

                if (newAngleDeg >= ShooterMinAngle && newAngleDeg <= ShooterMaxAngle) {
                    optimalAngleDeg = newAngleDeg;
                    optimalAngle = newAngleRad;
                    requiredVelocity = v_max;

                    rpm = ShooterMaxRPM; // since we are using max velocity
                } else {
                    // No valid angle or velocity so sotm defaults to largest angle and max rpm
                    return new ShooterSolution(ShooterMaxAngle, ShooterMaxRPM, launchAngle);
                }
            } 
        }
        
            return new ShooterSolution(optimalAngleDeg, rpm, launchAngle);
    }

    //i don't think I did this right but basically calculate flight time to find horizontal skew 
    public static double calculateTimeOfFlight(double horizontalDistanceMeters, double launchAngleDeg, double rpm) {
        double angleRad = Math.toRadians(launchAngleDeg);

        double wheelRadius = 0.0508; 
        double exitVelocity = (rpm / 60.0) * (2 * Math.PI * wheelRadius);

        double horizontalVelocity = exitVelocity * Math.cos(angleRad);
        if (horizontalVelocity <= 0) {
            return -1;
        }

        return horizontalDistanceMeters / horizontalVelocity;
    }

    //calculate the angle the robot needs to turn to compensate for sideways movement
    public static double calculateTurnAngle(double robotSidewaysVelocityMetersPerSec, double exitVelocityMetersPerSec) {
        
        return Math.atan(robotSidewaysVelocityMetersPerSec / exitVelocityMetersPerSec);

    }
    @Override
    public void close() {
        angleDeg.close();
        rpm.close();
        TurnAngleDeg.close();
    }
   
}