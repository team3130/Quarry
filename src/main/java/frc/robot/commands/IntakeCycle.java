// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCycle extends Command {
  /** Creates a new IntakeCycle. */
  private final Shooter shooter;
  private final Intake intake;
  private final Feeder feeder;
  public IntakeCycle(Shooter shooter, Intake intake, Feeder feeder) {
    this.shooter = shooter;
    this.intake = intake;
    this.feeder = feeder;
    addRequirements(feeder);
    addRequirements(intake);
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.runIntake();
    feeder.runFeederCycle();
    shooter.runShooterCycle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
