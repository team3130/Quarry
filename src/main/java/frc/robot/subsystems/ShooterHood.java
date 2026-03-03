// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterHood extends SubsystemBase {
  private final TalonFX hood;

  private final DigitalInput limit;
  private boolean isZeroed = false;

  private final MotionMagicVoltage voltRequest;
  private final TalonFXConfiguration motorConfig;

  private final Slot0Configs config;
  private double kP = 50;
  private double kI = 0;
  private double kD = 0;

  private double sensorToMechGearRatio = 208.842;

  private double targetAcceleration = 100;
  private double targetVelocity = 20;
  /** Creates a new ShooterHood. */
  public ShooterHood() {
    hood = new TalonFX(Constants.CAN.shooterHood);
    limit = new DigitalInput(Constants.IDs.shooterHoodLimit);

    config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;

    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive);
    motorConfig.MotionMagic = new MotionMagicConfigs()
        .withMotionMagicAcceleration(targetAcceleration)
        .withMotionMagicCruiseVelocity(targetVelocity);
    motorConfig.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(sensorToMechGearRatio);
    motorConfig.Slot0 = config;

    hood.getConfigurator().apply(motorConfig);

    voltRequest = new MotionMagicVoltage(0);
  }

  public void goToAngle(double setpoint) {
    hood.setControl(voltRequest.withPosition(setpoint));  
  }

  public void hoodUp(double speed) {
    hood.set(speed);
  }

  public void hoodDown(double positiveSpeed) {
    hood.set(-positiveSpeed);
  }

  public void stopHood() {
    hood.set(0);
  }

  public double getkP() {return kP;}
  public double getkI() {return kI;}
  public double getkD() {return kD;}
  public void setkP(double value) {kP = value;}
  public void setkI(double value) {kI = value;}
  public void setkD(double value) {kD = value;}

  public void updatePID() {
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    motorConfig.Slot0 = config;
    hood.getConfigurator().apply(motorConfig);
  }
  
  public double getPosition() {return hood.getPosition().getValueAsDouble();}
  public double getVelocity() {return hood.getVelocity().getValueAsDouble();}
  public double getAcceleration() {return hood.getAcceleration().getValueAsDouble();}

  public double getTargetAcceleration() {return targetAcceleration;}
  public void setTargetAcceleration(double value) {targetAcceleration = value;}

  public double getTargetVelocity() {return targetVelocity;}
  public void setTargetVelocity(double value) {targetVelocity = value;}

  public double getProfilePosition() {
    return hood.getClosedLoopReference().getValueAsDouble();
  }
  public double getProfileVelocity() {
    return hood.getClosedLoopReferenceSlope().getValueAsDouble();
  }

  public double getStatorCurrent() {return hood.getStatorCurrent().getValueAsDouble();}

  public double getGearRatio() {return sensorToMechGearRatio;}
  public void setGearRatio(double value) {sensorToMechGearRatio = value;}

  public boolean limitReached() {return limit.get();}
  
  public boolean isZeroed() {return isZeroed;}
  public void setZeroed(boolean value) {isZeroed = value;}

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Shooter Hood");

    builder.addDoubleProperty("Position", this::getPosition, null);

    builder.addBooleanProperty("Limit Reached", this::limitReached, null);
    builder.addBooleanProperty("Is Zeroed", this::isZeroed, this::setZeroed);

    builder.addDoubleProperty("Velocity (rot/s)", this::getVelocity, null);
    builder.addDoubleProperty("Acceleration (rot/s^2)", this::getAcceleration, null);

    builder.addDoubleProperty("Target Acceleration (rot/s^2)", this::getTargetAcceleration, this::setTargetAcceleration);
    builder.addDoubleProperty("Target Velocity (rot/s)", this::getTargetVelocity, this::setTargetVelocity);

    builder.addDoubleProperty("Profile Velocity (rot/s)", this::getProfileVelocity, null);
    builder.addDoubleProperty("Profile Position (rot)", this::getProfilePosition, null);

    builder.addDoubleProperty("Stator Current (A)", this::getStatorCurrent, null);

    builder.addDoubleProperty("Sensor to Mech Gear Ratio", this::getGearRatio, this::setGearRatio);

    builder.addDoubleProperty("kP", this::getkP, this::setkP);
    builder.addDoubleProperty("kI", this::getkI, this::setkI);
    builder.addDoubleProperty("kD", this::getkD, this::setkD);
  }

  @Override
  public void periodic() {
    if(limitReached()) {
      hood.setPosition(0);
      setZeroed(true);
    }
    // This method will be called once per scheduler run
  }
}
