// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
  private final TalonFX hopper;
  private double horizontalSpeed = 0.6;

  private final MotionMagicVelocityVoltage voltRequest;
  private final TalonFXConfiguration motorConfig;

  private final Slot0Configs config;
  private double kV = 0.12;
  private double kA = 0;
  private double kP = 0;
  private double kI = 0;
  private double kD = 0;

  private double sensorToMechGearRatio = 1;
  private double targetVelocity = 20;
  private double targetAcceleration = 100;
  /** Creates a new Hopper. */
  public Hopper() {
    hopper = new TalonFX(Constants.CAN.hopper);
 
    config = new Slot0Configs();
    config.kV = kV;
    config.kA = kA;
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;

    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive);
    motorConfig.MotionMagic = new MotionMagicConfigs().withMotionMagicAcceleration(targetAcceleration);
    motorConfig.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(sensorToMechGearRatio);
    motorConfig.Slot0 = config;

    hopper.getConfigurator().apply(motorConfig);

    voltRequest = new MotionMagicVelocityVoltage(0);
  }

  public void runHopperHorizontal() {hopper.set(horizontalSpeed);}
  public void reverseHopperHorizontal() {hopper.set(-horizontalSpeed);}
  public void stopHopperHorizontal() {hopper.set(0);}

  public void runHopperAtVelocity(double velocity) {
    hopper.setControl(voltRequest.withVelocity(velocity));
  }
  public void runHopper() {
    hopper.setControl(voltRequest.withVelocity(targetVelocity));
  }

  public double getkV() {return kV;}
  public double getkA() {return kA;}
  public double getkP() {return kP;}
  public double getkI() {return kI;}
  public double getkD() {return kD;}
  public void setkV(double value) {kV = value;}
  public void setkA(double value) {kA = value;}
  public void setkP(double value) {kP = value;}
  public void setkI(double value) {kI = value;}
  public void setkD(double value) {kD = value;}

  public void updatePID() {
    config.kV = kV;
    config.kA = kA;
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    motorConfig.Slot0 = config;
    hopper.getConfigurator().apply(motorConfig);
  }

  public double getVelocity() {return hopper.getVelocity().getValueAsDouble();}
  public double getAcceleration() {return hopper.getAcceleration().getValueAsDouble();}

  public double getProfileVelocity() {return hopper.getClosedLoopReference().getValueAsDouble();}
  public double getProfileAcceleration() {return hopper.getClosedLoopReferenceSlope().getValueAsDouble();}

  public double getTargetVelocity() {return targetVelocity;}
  public double getTargetAcceleration() {return targetAcceleration;}
  public void setTargetVelocity(double value) {targetVelocity = value;}
  public void setTargetAcceleration(double value) {targetAcceleration = value;}

  public double getStatorCurrent() {return hopper.getStatorCurrent().getValueAsDouble();}

  public double getGearRatio() {return sensorToMechGearRatio;}
  public void setGearRatio(double value) {sensorToMechGearRatio = value;}

  public boolean isAtVelocity() {return Math.abs(getVelocity() - getTargetVelocity()) < 0.1;}

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Hopper");

    builder.addDoubleProperty("Velocity (rot/s)", this::getVelocity, null);
    builder.addDoubleProperty("Acceleration (rot/s^2)", this::getAcceleration, null);

    builder.addDoubleProperty("Target Acceleration (rot/s^2)", this::getTargetAcceleration, this::setTargetAcceleration);
    builder.addDoubleProperty("Target Velocity (rot/s)", this::getTargetVelocity, this::setTargetVelocity);

    builder.addDoubleProperty("Profile Velocity (rot/s)", this::getProfileVelocity, null);
    builder.addDoubleProperty("Profile Acceleration (rot/s^2)", this::getProfileAcceleration, null);

    builder.addDoubleProperty("Stator Current (A)", this::getStatorCurrent, null);

    builder.addDoubleProperty("Sensor to Mech Gear Ratio", this::getGearRatio, this::setGearRatio);

    builder.addDoubleProperty("kV", this::getkV, this::setkV);
    builder.addDoubleProperty("kA", this::getkA, this::setkA);
    builder.addDoubleProperty("kP", this::getkP, this::setkP);
    builder.addDoubleProperty("kI", this::getkI, this::setkI);
    builder.addDoubleProperty("kD", this::getkD, this::setkD);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
