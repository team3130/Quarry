// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
  private final CommandSwerveDrivetrain driveTrain;

  private final String limelightName = "Limelight";
  
  /** Creates a new Limelight. */
  public Limelight(CommandSwerveDrivetrain driveTrain) {
    this.driveTrain = driveTrain;
    LimelightHelpers.setCameraPose_RobotSpace("", 0, 0, 0, 0, 0, 0);
  }

  public LimelightHelpers.PoseEstimate getRobotPose() {
    double robotYaw = driveTrain.getState().Pose.getRotation().getDegrees();
    LimelightHelpers.SetRobotOrientation("", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
