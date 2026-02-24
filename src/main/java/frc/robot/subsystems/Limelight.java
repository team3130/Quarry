// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
  private final CommandSwerveDrivetrain driveTrain;

  private final String limelightLeftName = "limelight-left";
  private final String limelightRightName = "limelight-right";

  
  /** Creates a new Limelight. */
  public Limelight(CommandSwerveDrivetrain driveTrain) {
    this.driveTrain = driveTrain;

    Rotation3d leftRot = new Rotation3d(Math.PI, 0, 0);
    leftRot = leftRot.rotateBy(new Rotation3d(0, 0, -Math.PI/6));
    leftRot = leftRot.rotateBy(new Rotation3d(0, -Math.PI/12, 0));

    Rotation3d rightRot = new Rotation3d(Math.PI, 0, 0);
    rightRot = rightRot.rotateBy(new Rotation3d(0, 0, Math.PI/6));
    rightRot = rightRot.rotateBy(new Rotation3d(0, -Math.PI/12, 0));

    LimelightHelpers.setCameraPose_RobotSpace(limelightLeftName, Units.inchesToMeters(5), Units.inchesToMeters(7.4), Units.inchesToMeters(25.5), leftRot.getX(), leftRot.getY(), leftRot.getZ());
    LimelightHelpers.setCameraPose_RobotSpace(limelightRightName, Units.inchesToMeters(5), Units.inchesToMeters(-7.4), Units.inchesToMeters(25.5), rightRot.getX(), rightRot.getY(), rightRot.getZ());
  }

  public LimelightHelpers.PoseEstimate getRobotPose(String choice) {
    String name = "";
    if(choice.equals("left")) {
      name = limelightLeftName;
    } else if(choice.equals("right")) {
      name = limelightRightName;
    }
    double robotYaw = driveTrain.getState().Pose.getRotation().getDegrees();
    LimelightHelpers.SetRobotOrientation(name, robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
