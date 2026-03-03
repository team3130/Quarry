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
import frc.robot.PowerAccount;
import frc.robot.PowerBank;

public class Hopper extends SubsystemBase {
  private final TalonFX horizontalHopper;
  private double horizontalSpeed = 0.8;
  private double hopperPower = 1; //Figure out what this should be after testing
  PowerAccount hopperAccount = PowerBank.getInstance().openAccount("Hopper", 1);
  private boolean isRunning = false;
  /** Creates a new Hopper. */

  public Hopper() {
    horizontalHopper = new TalonFX(Constants.CAN.hopper);
 
    horizontalHopper.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive)));
  }

  public void runHopperHorizontal() {
    isRunning = true;
  }

  public void reverseHopperHorizontal() {
    horizontalHopper.set(-horizontalSpeed);
  }

  public void stopHopperHorizontal() {
    hopperAccount.setMinRequest(0);
    hopperAccount.setMaxRequest(0);
    horizontalHopper.set(0);
  }
  
  @Override
  public void periodic() {
    if (isRunning) {
      hopperAccount.setMinRequest(hopperPower);
      hopperAccount.setMaxRequest(hopperPower);
      horizontalHopper.set(horizontalSpeed);
    }
  }
}
