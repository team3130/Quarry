package frc.robot.subsystems;

public class ShooterHub {
    double h = 1.8288; // height of hub in meters
    double v = 11.0; // initial velocity of ball in m/s
    public double findAngle(double D) {
        double g = 9.81; // acceleration due to gravity in m/s^2
        double angle = (2*Math.PI-Math.acos(-(h+g*D/v*v)/(Math.sqrt(h*h+D*D)))-Math.atan(D/h))/2;
        return angle;
    }
}


    
