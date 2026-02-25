// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter.PID;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.brains.ShooterMath;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RevToInterpolVel extends Command {
  private final Shooter shooter;
  private final ShooterMath shooterMath;
  
  /** Creates a new ShootAtVelocity. */
  public RevToInterpolVel(Shooter shooter, ShooterMath shooterMath) {
    this.shooter = shooter;
    this.shooterMath = shooterMath;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocity = shooterMath.RevInterpol();
    shooter.revAtVelocity(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
