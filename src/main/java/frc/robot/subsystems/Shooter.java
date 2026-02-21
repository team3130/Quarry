// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final TalonFX leftShooter;
  private final TalonFX rightShooter;

  private final MotionMagicVelocityVoltage voltRequest;
  private final TalonFXConfiguration motorConfig;

  private final Slot0Configs config;
  private double kV = 0.12;
  private double kA = 0;
  private double kP = 0;
  private double kI = 0;
  private double kD = 0;

  private double sensorToMechGearRatio = 1;

  private double accelerationMetersPerSecSquared = 5;
  private final double accelerationRotations = Units.radiansToRotations(accelerationMetersPerSecSquared/Units.inchesToMeters(2));

  private double targetVelocityMetersPerSec = 15;
  private final double targetVelocityRotations = Units.radiansToRotations(targetVelocityMetersPerSec/Units.inchesToMeters(2));

  private double speed = 1;
  /** Creates a new Shooter. */
  public Shooter() {
    rightShooter = new TalonFX(Constants.CAN.shooterRight);
    leftShooter = new TalonFX(Constants.CAN.shooterLeft);

    leftShooter.setControl(new Follower(rightShooter.getDeviceID(), MotorAlignmentValue.Opposed));

    config = new Slot0Configs();
    config.kV = kV;
    config.kA = kA;
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;

    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive);
    motorConfig.MotionMagic = new MotionMagicConfigs().withMotionMagicAcceleration(accelerationRotations);
    motorConfig.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(sensorToMechGearRatio);
    motorConfig.Slot0 = config;

    rightShooter.getConfigurator().apply(motorConfig);

    voltRequest = new MotionMagicVelocityVoltage(0);
  }

  public void revAtVelocity(double velocityMetersPerSec) {
    double radsPerSec = velocityMetersPerSec / Units.inchesToMeters(2);
    double rotsPerSec = Units.radiansToRotations(radsPerSec);
    rightShooter.setControl(voltRequest.withVelocity(rotsPerSec));
  }

  public void rev() {
    rightShooter.setControl(voltRequest.withVelocity(targetVelocityRotations));
  }

  public void runShooter() {
    rightShooter.set(speed);
  }

  public void reverseShooter() {
    rightShooter.set(-speed);
  }

  public void stopShooter() {
    rightShooter.set(0);
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
    rightShooter.getConfigurator().apply(motorConfig);
  }
  
  public double getVelocity() {
    double rotsPerSec = rightShooter.getVelocity().getValueAsDouble();
    double radsPerSec = Units.rotationsToRadians(rotsPerSec);
    double metersPerSec = radsPerSec * Units.inchesToMeters(2);
    return metersPerSec;
  }
  public double getAcceleration() {
    double rotsPerSecSquared = rightShooter.getAcceleration().getValueAsDouble();
    double radsPerSecSquared = Units.rotationsToRadians(rotsPerSecSquared);
    double metersPerSecSquared = radsPerSecSquared * Units.inchesToMeters(2);
    return metersPerSecSquared;
  }
  
  public double getTargetAcceleration() {return accelerationMetersPerSecSquared;}
  public void setTargetAcceleration(double value) {accelerationMetersPerSecSquared = value;}

  public double getTargetVelocity() {return targetVelocityMetersPerSec;}
  public void setTargetVelocity(double value) {targetVelocityMetersPerSec = value;}

  public double getProfileVelocity() {
    double rotsPerSec = rightShooter.getClosedLoopReference().getValueAsDouble();
    double radsPerSec = Units.rotationsToRadians(rotsPerSec);
    double metersPerSec = radsPerSec * Units.inchesToMeters(2);
    return metersPerSec;
  }
  public double getProfileAcceleration() {
    double rotsPerSecSquared = rightShooter.getClosedLoopReferenceSlope().getValueAsDouble();
    double radsPerSecSquared = Units.rotationsToRadians(rotsPerSecSquared);
    double metersPerSecSquared = radsPerSecSquared * Units.inchesToMeters(2);
    return metersPerSecSquared;
  }

  public double getStatorCurrent() {return rightShooter.getStatorCurrent().getValueAsDouble();}

  public double getGearRatio() {return sensorToMechGearRatio;}
  public void setGearRatio(double value) {sensorToMechGearRatio = value;}

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Shooter");

    builder.addDoubleProperty("Velocity (m/s)", this::getVelocity, null);
    builder.addDoubleProperty("Acceleration (m/s^2)", this::getAcceleration, null);

    builder.addDoubleProperty("Target Acceleration (m/s^2)", this::getTargetAcceleration, this::setTargetAcceleration);
    builder.addDoubleProperty("Target Velocity (m/s)", this::getTargetVelocity, this::setTargetVelocity);

    builder.addDoubleProperty("Profile Velocity", this::getProfileVelocity, null);
    builder.addDoubleProperty("Profile Acceleration", this::getProfileAcceleration, null);

    builder.addDoubleProperty("Stator Current", this::getStatorCurrent, null);

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
    //updatePID();
  }
}
