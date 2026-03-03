// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterHood.PID;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.brains.ShooterMath;
import frc.robot.subsystems.ShooterHood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoodToInterpol extends Command {
  private final double angle = 0.05;
  private final ShooterHood shooterHood;
  private final ShooterMath shooterMath;
  /** Creates a new HoodToSetpoint. */
  public HoodToInterpol(ShooterHood shooterHood, ShooterMath shooterMath) {
    this.shooterHood = shooterHood;
    this.shooterMath = shooterMath;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterHood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterHood.updatePID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = shooterMath.AngleInterpolNew();
    shooterHood.goToAngle(angle);
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
