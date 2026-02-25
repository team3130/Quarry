// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class InterpolationDoubleTree {
    InterpolatingDoubleTreeMap tableAngle = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap tableVel = new InterpolatingDoubleTreeMap();
    private final double hubHeight = 1;

    private static final double[][] measurments = {
        {0,1,2,3,4}, //distances
        {3,3,3,3,3}, //hoodMeasurements
        {3,3,3,3,3}, //shooterMeasurements
        {2,2,2,2,2}  //Time for Arcs
    };

    public double angleLinearizer(double angle, double speed) {
        double term1 = (2*9.8*hubHeight)/(speed*speed);
        double term2 = (Math.sin(angle)+Math.sqrt(Math.sin(angle)*Math.sin(angle)-term1));
        return term2*Math.cos(angle);
    }

    public double velocityLinearizer(double speed) {
        return speed*speed;
    }

    private final double[][] Intermeasurements = {
        {angleLinearizer(measurments[1][0], measurments[2][0]),angleLinearizer(measurments[1][1], measurments[2][1])
            ,angleLinearizer(measurments[1][2], measurments[2][2]),angleLinearizer(measurments[1][3], measurments[2][3])
            ,angleLinearizer(measurments[1][4], measurments[2][4])},//hoodMeasurements
        {velocityLinearizer(measurments[2][0]),velocityLinearizer(measurments[2][1])
            ,velocityLinearizer(measurments[2][2]),velocityLinearizer(measurments[2][3])
            ,velocityLinearizer(measurments[2][4])} //shooterMeasurements
    };

    public InterpolationDoubleTree() {
        //Key = Distance
        //Value = Angle
        tableAngle.put(measurments[0][0], Intermeasurements[0][0]);
        tableAngle.put(measurments[0][1], Intermeasurements[0][1]);
        tableAngle.put(measurments[0][2], Intermeasurements[0][2]);
        tableAngle.put(measurments[0][3], Intermeasurements[0][3]);
        tableAngle.put(measurments[0][4], Intermeasurements[0][4]);

        //Key = Distance
        //Value = Speed
        tableVel.put(measurments[0][0], Intermeasurements[1][0]);
        tableVel.put(measurments[0][1], Intermeasurements[1][1]);
        tableVel.put(measurments[0][2], Intermeasurements[1][2]);
        tableVel.put(measurments[0][3], Intermeasurements[1][3]);
        tableVel.put(measurments[0][4], Intermeasurements[1][4]);
    }

    public double getInterPolAngle(double distance, double velocity) {
        double term1 = Math.sqrt(1-Math.pow(distance, 2)-((19.6)/(velocity*velocity)));
        double term2 = (1+term1)/distance;
        return Math.acos(term2);
    }

    public double getInterPolVel(double distance) {
        return Math.sqrt(tableVel.get(distance));
    }

}
