// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.brains;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ShooterMath {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;
    private final double radius = Units.inchesToMeters(2);

    //New Measurment Arrays
    private static final double[] distances = {1,3.5};//,0,0,0};  //meters
    private static final double[] angles = {0,0.5};//,0,0,0};     //rots from position zero
    private static final double[] velocities = {16,16};//,0,0,0}; //meters per seconds
    private static final double[] times = {0,0};//,0,0,0};        //seconds

    private final double[] linearizeVel = {velocityLinearizer(velocities[0]),velocityLinearizer(velocities[1])};

    //Interpolation Objects
    InterpolatingDoubleTreeMap tableAngle = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap tableVel = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap tableVelLin = new InterpolatingDoubleTreeMap();

    public void InterpolationDoubleTree(){
        //Interpolation Double tree for Velocities
        tableVel.put(distances[0], velocities[0]);
        tableVel.put(distances[1], velocities[1]);
        //tableVel.put(distances[2], velocities[2]);
        //tableVel.put(distances[3], velocities[3]);
        //tableVel.put(distances[4], velocities[4]);

        //Interpolation Double tree for Angles
        tableAngle.put(distances[0], angles[0]);
        tableAngle.put(distances[1], angles[1]);
        //tableAngle.put(distances[2], angles[2]);
        //tableAngle.put(distances[3], angles[3]);
        //tableAngle.put(distances[4], angles[4]);

        //Linearized Velocity Table
        tableVelLin.put(distances[0], linearizeVel[0]);
        tableVelLin.put(distances[1], linearizeVel[1]);
    }

    public double velocityLinearizer(double speed) {return speed*speed;}

    public double getInterPolVel() {
        return Math.sqrt(tableVelLin.get(getDistanceFromHub()));
    }
    
    public ShooterMath(CommandSwerveDrivetrain commandSwerveDrivetrain) {
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    }

    
    public double getDistanceFromHub() {
        Translation2d originToBlueHub = new Translation2d(Units.inchesToMeters(181.56),Units.inchesToMeters(158.32));
        double distanceBlue = commandSwerveDrivetrain.getStatePose().getTranslation().getDistance(originToBlueHub);
        Translation2d originToRedHub = new Translation2d(Units.inchesToMeters(181.56+287),Units.inchesToMeters(158.32));
        double distanceRed = commandSwerveDrivetrain.getStatePose().getTranslation().getDistance(originToRedHub);

        return distanceBlue;
    }

    //Umar's Interpolation Request for Velocity
    public double RevInterpolNew() {
        double velmps = tableVel.get(getDistanceFromHub());
        double radspersec = velmps/(radius);
        double rotspersec = Units.radiansToRotations(radspersec);
        return rotspersec;
    }

    //Umar's Interpolation Request for Angle
    public double AngleInterpolNew() {
        return tableAngle.get(getDistanceFromHub());
    }

    public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Shooter Math");
    builder.addDoubleProperty("Distance from Hub", this::getDistanceFromHub, null);
    }
}