package frc.robot.commands.Chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveDistancePID extends Command {
    private final CommandSwerveDrivetrain driveTrain;
    private final SwerveRequest.FieldCentric drive;
    
    // PID Controller: P=2.0 is a solid start, I and D are added for stability
    private final PIDController pid = new PIDController(2.0, 0.0, 0.05); 
    
    private Pose2d startPose;
    private final double targetDistance = 3.0; // Meters to travel

    public DriveDistancePID(CommandSwerveDrivetrain driveTrain, SwerveRequest.FieldCentric drive) {
        this.driveTrain = driveTrain;
        this.drive = drive;
        
        // 5cm tolerance is usually tight enough for a 3m sprint
        pid.setTolerance(0.05); 
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        // Capture the exact starting position and rotation
        startPose = driveTrain.getState().Pose;
        
        // Reset the PID internal state (crucial if you have any 'I' value)
        pid.reset();
    }

    @Override
    public void execute() {
        Pose2d currentPose = driveTrain.getState().Pose;

        // Calculate straight-line distance from the starting point
        double distanceTraveled = currentPose.getTranslation().getDistance(startPose.getTranslation());

        // PID calculates the velocity magnitude needed to reach targetDistance
        double velocityMagnitude = pid.calculate(distanceTraveled, targetDistance);

        // Break the velocity magnitude into X and Y components based on starting angle
        // This ensures the robot drives "straight" relative to its initial orientation
        double velocityX = velocityMagnitude * startPose.getRotation().getCos();
        double velocityY = velocityMagnitude * startPose.getRotation().getSin();

        driveTrain.setControl(drive
            .withVelocityX(velocityX)
            .withVelocityY(velocityY)
            .withRotationalRate(0));
    }

    @Override
    public void end(boolean interrupted) {
        // Neutral out the drive motors
        driveTrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        // End when within tolerance
        return pid.atSetpoint();
    }
}