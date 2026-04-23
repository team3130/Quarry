// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake.PID;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotHalf extends Command {
  private final Intake intake;
  private final Timer timer;
  private final double increment = 0.0333;
  private double currentPos = 0.15;
  private double startPos = 0.13055;
  private final double maxPos = 0.03;
  private boolean incremented = false;
  /** Creates a new PivotHalf. */
  public PivotHalf(Intake intake) {
    this.intake = intake;
    this.timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    startPos = 0.15;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() > 1.3) {
      timer.restart();
      incremented = false;
    } else if(timer.get() > 1) {
      intake.intakePivotToSetpoint(0.12);
      if((startPos - increment) >= maxPos && !incremented) {
        startPos -= increment;
        incremented = true;
      } else if(!incremented) {
        startPos = maxPos;
      }
    } else {
      intake.intakePivotToSetpoint(startPos);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeOut();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
