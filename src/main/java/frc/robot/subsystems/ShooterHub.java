package frc.robot.subsystems;

public class ShooterHub {
    double h; // height of hub in meters
    double v; // initial velocity of ball in m/s

    public ShooterHub(double h, double v) {
        this.h = h;
        this.v = v;
    }

    public double findAngle(double D) {
        double g = 9.81; // acceleration due to gravity in m/s^2
        double angle = (2*Math.PI-Math.acos(-(h+g*D/v*v)/(Math.sqrt(h*h+D*D)))-Math.atan(D/h))/2;
        return angle;
    }
}


    
