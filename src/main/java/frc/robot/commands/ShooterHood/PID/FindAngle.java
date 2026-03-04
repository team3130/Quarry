// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterHood.PID;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.ShooterHub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FindAngle extends Command {
  /** Creates a new FindAngle. */
  private final CommandSwerveDrivetrain drivetrain;
  private final ShooterHub shooterHub;
  private final ShooterHood shooterHood;
  private final Translation2d hubVector = new Translation2d(3.9, 0);
  public FindAngle(ShooterHub shooterHub, ShooterHood shooterHood, CommandSwerveDrivetrain drivetrain) {
    this.shooterHub = shooterHub;
    this.shooterHood = shooterHood;
    this.drivetrain = drivetrain;
    addRequirements(shooterHood);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterHood.updatePID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d odoVec = drivetrain.getState().Pose.getTranslation();
    Translation2d targetVector = hubVector.minus(odoVec);
    double distance = targetVector.getNorm();
    double angle = shooterHub.findAngle(distance);
    if(0 <= angle && 0.1 >= angle) {
      shooterHood.goToAngle(distance);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterHood.stopHood();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
