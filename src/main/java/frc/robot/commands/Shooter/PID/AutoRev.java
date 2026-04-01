// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter.PID;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoRev extends Command {
  private final Shooter shooter;
  private final CommandSwerveDrivetrain drivetrain;
  private final ShooterHood shooterHood;
  /** Creates a new Shoot. */
  public AutoRev(Shooter shooter, CommandSwerveDrivetrain drivetrain, ShooterHood shooterHood) {
    this.shooter = shooter;
    this.drivetrain = drivetrain;
    this.shooterHood = shooterHood;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setIsShooting(true);
    //shooter.updatePID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.autoRev(drivetrain);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setIsShooting(false);
    shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
