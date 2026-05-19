// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
  private boolean robotHeadingReset = false;

  private final String limelightLeftName = "limelight-left";
  private final String limelightRightName = "limelight-right";

  
  /** Creates a new Limelight. */
  public Limelight() {
    Rotation3d leftRot = new Rotation3d(Math.PI, 0, 0);
    leftRot = leftRot.rotateBy(new Rotation3d(0, 0, -Math.PI/6));
    leftRot = leftRot.rotateBy(new Rotation3d(0, Math.PI/12, 0));

    Rotation3d rightRot = new Rotation3d(Math.PI, 0, 0);
    rightRot = rightRot.rotateBy(new Rotation3d(0, 0, Math.PI/6));
    rightRot = rightRot.rotateBy(new Rotation3d(0, Math.PI/12, 0));

    LimelightHelpers.setCameraPose_RobotSpace(limelightLeftName, Units.inchesToMeters(5), Units.inchesToMeters(-7.4), Units.inchesToMeters(25.5), Units.radiansToDegrees(leftRot.getX()), Units.radiansToDegrees(leftRot.getY()), Units.radiansToDegrees(leftRot.getZ()));
    LimelightHelpers.setCameraPose_RobotSpace(limelightRightName, Units.inchesToMeters(5), Units.inchesToMeters(7.4), Units.inchesToMeters(25.5), Units.radiansToDegrees(rightRot.getX()), Units.radiansToDegrees(rightRot.getY()), Units.radiansToDegrees(rightRot.getZ()));
  }

  public LimelightHelpers.PoseEstimate getRobotPose(String name, CommandSwerveDrivetrain driveTrain) {
    double robotYaw = driveTrain.getState().Pose.getRotation().getDegrees();
    LimelightHelpers.SetRobotOrientation(name, robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
  }

  public LimelightHelpers.PoseEstimate getMT1RobotPose(String name) {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
  }

  public void updateOdo(CommandSwerveDrivetrain driveTrain) {
    if(robotHeadingReset) {
      LimelightHelpers.PoseEstimate leftPose = getRobotPose(limelightLeftName, driveTrain);
      LimelightHelpers.PoseEstimate rightPose = getRobotPose(limelightRightName, driveTrain);
      if(leftPose != null && leftPose.tagCount > 0) {
        driveTrain.addVisionMeasurement(leftPose.pose, leftPose.timestampSeconds);
      }
      if(rightPose != null && rightPose.tagCount > 0) {
        driveTrain.addVisionMeasurement(rightPose.pose, rightPose.timestampSeconds);
      }
    }
  }

  public void updateDisabledOdo(CommandSwerveDrivetrain driveTrain) {
    if(!robotHeadingReset) {
      LimelightHelpers.PoseEstimate leftPose = getMT1RobotPose(limelightLeftName);
      LimelightHelpers.PoseEstimate rightPose = getMT1RobotPose(limelightRightName);
      if(leftPose != null && leftPose.tagCount > 1) {
        driveTrain.addVisionMeasurement(leftPose.pose, leftPose.timestampSeconds);
        driveTrain.resetRotation(leftPose.pose.getRotation());
      }
      if(rightPose != null && rightPose.tagCount > 1) {
        driveTrain.addVisionMeasurement(rightPose.pose, rightPose.timestampSeconds);
        driveTrain.resetRotation(rightPose.pose.getRotation());
      }
    }
  }

  public boolean IsRobotHeadingReset() {return IsRobotHeadingReset();}
  public void setRobotHeadingReset(boolean value) {robotHeadingReset = value;}

  public void disabledDeviations(CommandSwerveDrivetrain driveTrain) {
    driveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(1, 1, 0.3));
  }

  public void autonDeviations(CommandSwerveDrivetrain drivetrain) {
    drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(2, 2, 9999999));
  }

  public void enabledDeviations(CommandSwerveDrivetrain driveTrain) {
    driveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(1, 1, 9999999));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
