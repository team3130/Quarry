// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Feeder.PID;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunFeeder extends Command {
  private final Feeder feeder;
  private final Shooter shooter;
  private final ShooterHood shooterHood;
  private final CommandSwerveDrivetrain drivetrain;
  /** Creates a new RunFeeder. */
  public RunFeeder(Feeder feeder, Shooter shooter, ShooterHood shooterHood, CommandSwerveDrivetrain drivetrain) {
    this.feeder = feeder;
    this.shooter = shooter;
    this.shooterHood = shooterHood;
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //feeder.updatePID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.isAtVelocity()) {
      feeder.runFeeder();
    } else {
      feeder.stopFeeder();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.stopFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
