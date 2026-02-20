// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private final Solenoid bottomHooks;
  private final Solenoid topHooks;

  private boolean bottomExtended = false;
  private boolean topExtended = false;
  /** Creates a new Climber. */
  public Climber() {
    bottomHooks = new Solenoid(35, PneumaticsModuleType.CTREPCM, Constants.IDs.bottomHooks);
    topHooks = new Solenoid(Constants.CAN.PCM, PneumaticsModuleType.CTREPCM, Constants.IDs.topHooks);
  }

  public void extendBottomHooks() {bottomHooks.set(true);}
  public void retractBottomHooks() {bottomHooks.set(false);}

  public void extendTopHooks() {topHooks.set(true);}
  public void retractTopHooks() {topHooks.set(false);}

  public boolean isBottomExtended() {return bottomExtended;}
  public boolean isTopExtended() {return topExtended;}

  public void setBottomExtended(boolean value) {bottomExtended = value;}
  public void setTopExtended(boolean value) {topExtended = value;}

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Climber");

    builder.addBooleanProperty("Bottom Hooks Extended", this::isBottomExtended, this::setBottomExtended);
    builder.addBooleanProperty("Top Hooks Extended", this::isTopExtended, this::setTopExtended);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
