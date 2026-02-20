// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX intake;
  private double speed = 1;
  public Intake() {
    intake = new TalonFX(Constants.CAN.intake);
    intake.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive)));
  }

  public void runIntake() {
    intake.set(speed);
  }

  public void reverseIntake() {
    intake.set(-speed);
  }

  public void stopIntake() {
    intake.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
