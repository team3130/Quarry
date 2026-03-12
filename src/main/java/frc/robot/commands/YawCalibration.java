// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.apache.commons.math3.stat.regression.OLSMultipleLinearRegression;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Telemetry;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class YawCalibration extends Command {
  private Limelight limelight;
  private CommandSwerveDrivetrain driveTrain;
  private String limelightName = "limelight-left";
  private ArrayList<double[]> input = new ArrayList<>();
  /** Creates a new YawCalibration. */
  public YawCalibration(CommandSwerveDrivetrain driveTrain, Limelight limelight) {
      this.driveTrain = driveTrain;
      AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
      this.limelight = limelight;
      addRequirements(driveTrain);
      Rotation3d rotation = new Rotation3d(Math.PI, 0, 0);
      if(limelightName.equals("limelight-left")) {
        rotation = rotation.rotateBy(new Rotation3d(0, 0, -Math.PI/6));
        LimelightHelpers.setCameraPose_RobotSpace(limelightName, Units.inchesToMeters(5), Units.inchesToMeters(-7.4), Units.inchesToMeters(25.5), Units.radiansToDegrees(rotation.getX()), Units.radiansToDegrees(rotation.getY()), 0);
      } else if(limelightName.equals("limelight-right")) {
        rotation = rotation.rotateBy(new Rotation3d(0, 0, Math.PI/6));
        LimelightHelpers.setCameraPose_RobotSpace(limelightName, Units.inchesToMeters(5), Units.inchesToMeters(7.4), Units.inchesToMeters(25.5), Units.radiansToDegrees(rotation.getX()), Units.radiansToDegrees(rotation.getY()), 0);
      } 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        LimelightHelpers.PoseEstimate pose = limelight.getRobotPose(limelightName);
        if(pose != null) {
            double[] measurement = {pose.pose.getX(), pose.pose.getY()};
            input.add(measurement);
        }
    }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
 if (input.isEmpty()) {
            System.out.println("Calibration failed: No samples.");
            return;
        }

        double[][] X = new double[input.size()][2];
        double[] Z = new double[input.size()];
        for (int i = 0; i < input.size(); i++) {
            double[] p = input.get(i);
            X[i][0] = p[0];
            X[i][1] = 1.0;
            Z[i] = p[1]; 
        }

        OLSMultipleLinearRegression regression = new OLSMultipleLinearRegression();
        regression.newSampleData(Z, X);

        double[] params = regression.estimateRegressionParameters();
        double a = params[0];
        double c = params[1];

        double yaw = -Math.atan(a);

        System.out.println("RESULT");
        System.out.println("Slope = " +  a + "\n Yaw = " + yaw);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
