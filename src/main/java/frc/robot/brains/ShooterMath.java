// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.brains;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.InterpolationDoubleTree;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ShooterMath {
    private final InterpolationDoubleTree interpolationDoubleTree;
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;
    private final double radius = Units.inchesToMeters(2);

    //New Measurment Arrays
    private static final double[] distances = {1,3.5};//,0,0,0};  //meters
    private static final double[] angles = {0,0.5};//,0,0,0};     //rots from position zero
    private static final double[] velocities = {16,16};//,0,0,0}; //meters per seconds
    private static final double[] times = {0,0};//,0,0,0};        //seconds

    //Interpolation Objects
    InterpolatingDoubleTreeMap tableAngle = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap tableVel = new InterpolatingDoubleTreeMap();

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
    }
    
    public ShooterMath(InterpolationDoubleTree interpolationDoubleTree, CommandSwerveDrivetrain commandSwerveDrivetrain) {
        this.interpolationDoubleTree = interpolationDoubleTree;
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    }

    public double getDistanceFromHub() {
        Translation2d originToHub = new Translation2d(Units.inchesToMeters(181.56),Units.inchesToMeters(158.32));
        double distance = commandSwerveDrivetrain.getStatePose().getTranslation().getDistance(originToHub);
        return distance;
    }



    public double RevInterpol() {
        return interpolationDoubleTree.getInterPolVel(getDistanceFromHub());
    }

    public double AngleInterpol() {
        return interpolationDoubleTree.getInterPolAngle(getDistanceFromHub(), RevInterpol());
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
    builder.addDoubleProperty("Rev Interpol Speed", this::RevInterpol, null);
    }
}