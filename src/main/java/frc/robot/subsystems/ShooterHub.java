package frc.robot.subsystems;

public class ShooterHub {
    double h; // height of hub in meters
    double v; // initial velocity of ball in m/s

    public ShooterHub(double h, double v) {
        this.h = h;
        this.v = v;
    }

    // public double findAngle(double D) {
    //     double g = 9.81; // acceleration due to gravity in m/s^2
    //     double angle = (2*Math.PI-Math.acos(-(h+g*D/(v*v))/(Math.sqrt(h*h+D*D)))-Math.atan(D/h))/2;
    //     return angle;
    // }

    public double findLowAngle(double D) {
        double g = 9.81;
        double v2 = v * v;
        
        // The discriminant: determines if the target is within reach
        double discriminant = Math.pow(v2, 2) - g * (g * Math.pow(D, 2) + 2 * h * v2);
        
        if (discriminant < 0) {
            return Double.NaN; // Velocity is too low to reach the target
        }

        // Using the minus sign here for the flatter, "low" trajectory
        double tanTheta = (v2 - Math.sqrt(discriminant)) / (g * D);
        
        return Math.atan(tanTheta); 
    }
}


    
