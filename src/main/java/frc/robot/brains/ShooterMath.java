// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.brains;

import java.util.InterpolatingDoubleTreeMap.InterpolationDoubleTree;

/** Add your docs here. */
public class ShooterMath {
    private final InterpolationDoubleTree interpolationDoubleTree;

    public ShooterMath(InterpolationDoubleTree interpolationDoubleTree) {
        this.interpolationDoubleTree = interpolationDoubleTree;
    }

    private static final double[][] measurments = {
        {0,1,2,3,4}, //distances
        {3,3,3,3,3}, //hoodMeasurements
        {3,3,3,3,3}, //shooterMeasurements
        {2,2,2,2,2}  //Time for Arcs
    };
    


    public double getDistance() {return 4;}
    public double getVelocityFromHub() {return 0.5;}

    //This is just using distance: 1 input to 1 output
    public double Rev0Interpol(double distance) {
        if(distance >= measurments[0][0] && distance <= measurments[0][1]) {
            double slopeNum = (measurments[2][1]-measurments[2][0]);
            double slopeDen = (measurments[0][1]-measurments[0][0]);
            double slope = (slopeNum/slopeDen);
            double metersPerSec = (slope)*(distance-measurments[0][0])+measurments[2][0];
            return metersPerSec;
        }if(distance > measurments[0][1] && distance <= measurments[0][2]) {
            double slopeNum = (measurments[2][2]-measurments[2][1]);
            double slopeDen = (measurments[0][2]-measurments[0][1]);
            double slope = (slopeNum/slopeDen);
            double metersPerSec = (slope)*(distance-measurments[0][1])+measurments[2][1];
            return metersPerSec;
        }if(distance > measurments[0][2] && distance <= measurments[0][3]) {
            double slopeNum = (measurments[2][3]-measurments[2][2]);
            double slopeDen = (measurments[0][3]-measurments[0][2]);
            double slope = (slopeNum/slopeDen);
            double metersPerSec = (slope)*(distance-measurments[0][2])+measurments[2][2];
            return metersPerSec;
        }if(distance > measurments[0][3] && distance <= measurments[0][4]) {
            double slopeNum = (measurments[2][4]-measurments[2][3]);
            double slopeDen = (measurments[0][4]-measurments[0][3]);
            double slope = (slopeNum/slopeDen);
            double metersPerSec = (slope)*(distance-measurments[0][3])+measurments[2][3];
            return metersPerSec;
        }else{return 0;}
    }

    //This is just using distance: 1 input to 1 output
    public double Hood0Interpol(double distance) {
        if(distance >= measurments[0][0] && distance <= measurments[0][1]) {
            double slopeNum = (measurments[1][1]-measurments[1][0]);
            double slopeDen = (measurments[0][1]-measurments[0][0]);
            double slope = (slopeNum/slopeDen);
            double metersPerSec = (slope)*(distance-measurments[0][0])+measurments[1][0];
            return metersPerSec;
        }if(distance > measurments[0][1] && distance <= measurments[0][2]) {
            double slopeNum = (measurments[1][2]-measurments[1][1]);
            double slopeDen = (measurments[0][2]-measurments[0][1]);
            double slope = (slopeNum/slopeDen);
            double metersPerSec = (slope)*(distance-measurments[0][1])+measurments[1][1];
            return metersPerSec;
        }if(distance > measurments[0][2] && distance <= measurments[0][3]) {
            double slopeNum = (measurments[1][3]-measurments[1][2]);
            double slopeDen = (measurments[0][3]-measurments[0][2]);
            double slope = (slopeNum/slopeDen);
            double metersPerSec = (slope)*(distance-measurments[0][2])+measurments[1][2];
            return metersPerSec;
        }if(distance > measurments[0][3] && distance <= measurments[0][4]) {
            double slopeNum = (measurments[1][4]-measurments[1][3]);
            double slopeDen = (measurments[0][4]-measurments[0][3]);
            double slope = (slopeNum/slopeDen);
            double metersPerSec = (slope)*(distance-measurments[0][3])+measurments[1][3];
            return metersPerSec;
        }else{return 0;}
    }

    public double RevInterpol() {
        return interpolationDoubleTree.getInterPolVel(getDistance());
    }

    public double AngleInterpol() {
        return interpolationDoubleTree.getInterPolAngle(getDistance(), RevInterpol());
    }
}