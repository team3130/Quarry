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
import frc.robot.AccelLimiter;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopDrive extends Command {
  private final CommandSwerveDrivetrain driveTrain;
  private final Shooter shooter;
  private final ShooterHood shooterHood;
  private final CommandPS5Controller controller;
  private final AccelLimiter accelLimiter;
  private final SwerveRequest.FieldCentric drive;

  private Translation2d hubVector = new Translation2d(0, 0);

  /** Creates a new TeleopDrive. */
  public TeleopDrive(CommandSwerveDrivetrain driveTrain, CommandPS5Controller controller, SwerveRequest.FieldCentric drive,
  Shooter shooter, ShooterHood shooterHood) {
    this.driveTrain = driveTrain;
    this.shooter = shooter;
    this.shooterHood = shooterHood;
    this.controller = controller;
    this.drive = drive;
    accelLimiter = new AccelLimiter(15, -15, 0, 1000);
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
      double[] data = driveTrain.targetAnglesAndSpeeds(shooter, hubVector, controller);
      driveTrain.setControl(drive
                .withVelocityX(data[2])
                .withVelocityY(data[3])
                .withRotationalRate(data[0] + data[1]));
    } else {
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