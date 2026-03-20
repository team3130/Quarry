// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
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
    if(driveTrain.getHubToggle()) {
      Translation2d robotVector = driveTrain.getState().Pose.getTranslation();
      Translation2d targetVector = hubVector.minus(robotVector);
      double targetAngle = targetVector.getAngle().getDegrees();
      if(driveTrain.getIsShooting()) {
        targetAngle = driveTrain.getAngleSetpoint();
      }
      double robotAngle = driveTrain.getState().Pose.getRotation().getDegrees();
      if(targetAngle - robotAngle > 180) {
        targetAngle -= 360;
      } else if(targetAngle - robotAngle < -180) {
        targetAngle += 360;
      }
      System.out.println("Target Angle: " + targetAngle);
      double angleInput = pidController.calculate(robotAngle, targetAngle);
      driveTrain.setControl(drive
                .withVelocityX(driveTrain.applySingleDeadband(-controller.getLeftY(), maxSpeed))
                .withVelocityY(driveTrain.applySingleDeadband(-controller.getLeftX(), maxSpeed))
                .withRotationalRate(angleInput));
      } else {
      pidController.reset();
      ChassisSpeeds targetSpeeds = driveTrain.accelLimitVectorDrive(driveTrain.getHIDspeedsMPS(controller));
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
