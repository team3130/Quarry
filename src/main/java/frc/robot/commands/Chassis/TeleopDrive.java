// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopDrive extends Command {
  private final CommandSwerveDrivetrain driveTrain;
  private final CommandPS5Controller controller;
  private final double maxSpeed;
  private final double maxAngularRate;
  private final SwerveRequest.FieldCentric drive;

  private Translation2d hubVector = new Translation2d(0, 0);
  private final PIDController pidController;
  /** Creates a new TeleopDrive. */
  public TeleopDrive(CommandSwerveDrivetrain driveTrain, CommandPS5Controller controller, 
                    double maxSpeed, double maxAngularRate, 
                    SwerveRequest.FieldCentric drive) {
    this.driveTrain = driveTrain;
    this.controller = controller;
    this.maxSpeed = maxSpeed;
    this.maxAngularRate = maxAngularRate;
    this.drive = drive;
    pidController = new PIDController(0.05, 0.01, 0);
    pidController.enableContinuousInput(-180, 180);
    SmartDashboard.putData(pidController);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(DriverStation.getAlliance().isPresent()) {
      if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        hubVector = new Translation2d(Units.inchesToMeters(181.56),Units.inchesToMeters(158.32));
      } else {
        hubVector = new Translation2d(Units.inchesToMeters(181.56 + 287),Units.inchesToMeters(158.32));
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds targetSpeeds = driveTrain.accelLimitVectorDrive(driveTrain.getHIDspeedsMPS(controller));
    if(driveTrain.getHubToggle()) {
      Translation2d robotVector = driveTrain.getState().Pose.getTranslation();
      Translation2d targetVector = hubVector.minus(robotVector);
      double targetAngle = targetVector.getAngle().getDegrees();
      double robotAngle = driveTrain.getState().Pose.getRotation().getDegrees();
      if(targetAngle - robotAngle > 180) {
        targetAngle -= 360;
      } else if(targetAngle - robotAngle < -180) {
        targetAngle += 360;
      }
      // If shooting correct for movement, otherwise just target hub regardless of where you are
      if (driveTrain.getIsShooting()) {
          targetAngle = driveTrain.getAngleSetpoint(); 
      } else {
          targetAngle = hubVector.minus(robotVector).getAngle().getDegrees();
      }
      // Keep angles in the range (-180, 180]
      if(targetAngle - robotAngle > 180) {
        targetAngle -= 360;
      } else if(targetAngle - robotAngle < -180) {
        targetAngle += 360;
      }
      // Robot angle is within 3 degrees of target angle
      if(Math.abs(robotAngle - targetAngle) < 3) {
        driveTrain.setFacingTarget(true);
      } else {
        driveTrain.setFacingTarget(false);
      }
      double angleInput = pidController.calculate(robotAngle, targetAngle);

      var chassisState = driveTrain.getState(); // Get Speeds and Pose
      Translation2d robotFieldVel = new Translation2d(
          chassisState.Speeds.vxMetersPerSecond, 
          chassisState.Speeds.vyMetersPerSecond
      ).rotateBy(chassisState.Pose.getRotation()); // Field Relative Robot Velocity
      // Calculation of unit tangent vector. Take vector to hub, rotate by 90 degrees to get tangential vector, normalize tangential vector
      Translation2d unitTangent = targetVector.rotateBy(new Rotation2d(Math.PI/2)).div(targetVector.getNorm());
      // Angle correct the opposite direction of movement using w = -v/R
      double angleOutput = -unitTangent.dot(robotFieldVel)/targetVector.getNorm();

      driveTrain.setControl(drive
                .withVelocityX(targetSpeeds.vxMetersPerSecond)
                .withVelocityY(targetSpeeds.vyMetersPerSecond)
                .withRotationalRate(angleInput + angleOutput));
      } else {
      pidController.reset();
      driveTrain.setControl(drive
      .withVelocityX(targetSpeeds.vxMetersPerSecond)
      .withVelocityY(targetSpeeds.vyMetersPerSecond)
      .withRotationalRate(targetSpeeds.omegaRadiansPerSecond));
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
