// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Telemetry;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class UpdateOdoFromVision extends Command {
  private final CommandSwerveDrivetrain driveTrain;
  private final Limelight limelights;
  private final Telemetry telemetry;
  /** Creates a new UpdateOdoFromVision. */
  public UpdateOdoFromVision(CommandSwerveDrivetrain driveTrain, Limelight limelights, Telemetry telemetry) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.limelights = limelights;
    this.telemetry = telemetry;

    addRequirements(limelights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5, 9999999));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LimelightHelpers.PoseEstimate leftPose = limelights.getRobotPose("left");
    LimelightHelpers.PoseEstimate rightPose = limelights.getRobotPose("right");
    if(leftPose != null && leftPose.tagCount > 0) {
      driveTrain.addVisionMeasurement(leftPose.pose, leftPose.timestampSeconds);
    }
    if(rightPose != null && rightPose.tagCount > 0) {
      driveTrain.addVisionMeasurement(rightPose.pose, rightPose.timestampSeconds);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
