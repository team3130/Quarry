// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private final TalonFX elevator;

  private final MotionMagicVoltage voltRequest;
  private final TalonFXConfiguration motorConfig;

  private final Slot0Configs emptyConfig;
  private double slot0_kG = 0;
  private double slot0_kV = 0.12;
  private double slot0_kA = 0;
  private double slot0_kP = 0;
  private double slot0_kI = 0;
  private double slot0_kD = 0;

  private final Slot1Configs fullConfig;
  private double slot1_kG = 0;
  private double slot1_kV = 0.12;
  private double slot1_kA = 0;
  private double slot1_kP = 0;
  private double slot1_kI = 0;
  private double slot1_kD = 0;

  private double reachPos = 0;
  private double bottomPos = 0;

  private double sensorToMechGearRatio = 1;

  private double targetAcceleration = 2;
  private double targetVelocity = 2;

  private final DigitalInput limitSwitch;
  private boolean isZeroed = false;

  private boolean bottomExtended = false;
  private boolean topExtended = false;

  private final double speed = 0.1;
  /** Creates a new Climber. */
  public Climber() {
    elevator = new TalonFX(Constants.CAN.climberElevator);

    limitSwitch = new DigitalInput(Constants.IDs.climberLimit);

    emptyConfig = new Slot0Configs();
    emptyConfig.kG = slot0_kG;
    emptyConfig.kV = slot0_kV;
    emptyConfig.kA = slot0_kA;
    emptyConfig.kP = slot0_kP;
    emptyConfig.kI = slot0_kI;
    emptyConfig.kD = slot0_kI;

    fullConfig = new Slot1Configs();
    fullConfig.kG = slot1_kG;
    fullConfig.kV = slot1_kV;
    fullConfig.kA = slot1_kA;
    fullConfig.kP = slot1_kP;
    fullConfig.kI = slot1_kI;
    fullConfig.kD = slot1_kI;

    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);
    motorConfig.MotionMagic = new MotionMagicConfigs()
        .withMotionMagicAcceleration(targetAcceleration)
        .withMotionMagicCruiseVelocity(targetVelocity);
    motorConfig.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(sensorToMechGearRatio);
    motorConfig.Slot0 = emptyConfig;
    motorConfig.Slot1 = fullConfig;

    elevator.getConfigurator().apply(motorConfig);

    voltRequest = new MotionMagicVoltage(0);
  }

  public void climberUp() {
    elevator.set(speed);
  }
  public void climberDown() {
    elevator.set(-speed);
  }
  public void stopClimber() {
    elevator.set(0);
  }
  
  public void goToSetpoint(double setpoint) {
    elevator.setControl(voltRequest.withPosition(setpoint));
  }
  public void goToReach() {
    elevator.setControl(voltRequest.withPosition(reachPos));
  }
  public void goToBottom() {
    elevator.setControl(voltRequest.withPosition(bottomPos));
  }

  public double getSlot0_kG() {return slot0_kG;}
  public double getSlot0_kV() {return slot0_kV;}
  public double getSlot0_kA() {return slot0_kA;}
  public double getSlot0_kP() {return slot0_kP;}
  public double getSlot0_kI() {return slot0_kI;}
  public double getSlot0_kD() {return slot0_kD;}
  public void setSlot0_kG(double value) {slot0_kG = value;}
  public void setSlot0_kV(double value) {slot0_kV = value;}
  public void setSlot0_kA(double value) {slot0_kA = value;}
  public void setSlot0_kP(double value) {slot0_kP = value;}
  public void setSlot0_kI(double value) {slot0_kI = value;}
  public void setSlot0_kD(double value) {slot0_kD = value;}
  public double getSlot1_kG() {return slot1_kG;}
  public double getSlot1_kV() {return slot1_kV;}
  public double getSlot1_kA() {return slot1_kA;}
  public double getSlot1_kP() {return slot1_kP;}
  public double getSlot1_kI() {return slot1_kI;}
  public double getSlot1_kD() {return slot1_kD;}
  public void setSlot1_kG(double value) {slot1_kG = value;}
  public void setSlot1_kV(double value) {slot1_kV = value;}
  public void setSlot1_kA(double value) {slot1_kA = value;}
  public void setSlot1_kP(double value) {slot1_kP = value;}
  public void setSlot1_kI(double value) {slot1_kI = value;}
  public void setSlot1_kD(double value) {slot1_kD = value;}

  public void updatePID() {
    emptyConfig.kG = slot0_kG;
    emptyConfig.kV = slot0_kV;
    emptyConfig.kA = slot0_kA;
    emptyConfig.kP = slot0_kP;
    emptyConfig.kI = slot0_kI;
    emptyConfig.kD = slot0_kD;
    motorConfig.Slot0 = emptyConfig;

    fullConfig.kG = slot1_kG;
    fullConfig.kV = slot1_kV;
    fullConfig.kA = slot1_kA;
    fullConfig.kP = slot1_kP;
    fullConfig.kI = slot1_kI;
    fullConfig.kD = slot1_kD;
    motorConfig.Slot1 = fullConfig;

    elevator.getConfigurator().apply(motorConfig);
  }

  public double getVelocity() {return elevator.getVelocity().getValueAsDouble();}
  public double getAcceleration() {return elevator.getAcceleration().getValueAsDouble();}

  public double getTargetAcceleration() {return targetAcceleration;}
  public void setTargetAcceleration(double value) {targetAcceleration = value;}

  public double getTargetVelocity() {return targetVelocity;}
  public void setTargetVelocity(double value) {targetVelocity = value;}

  public double getProfileVelocity() {
    return elevator.getClosedLoopReference().getValueAsDouble();
  }
  public double getProfileAcceleration() {
    return elevator.getClosedLoopReferenceSlope().getValueAsDouble();
  }

  public double getStatorCurrent() {return elevator.getStatorCurrent().getValueAsDouble();}

  public double getGearRatio() {return sensorToMechGearRatio;}
  public void setGearRatio(double value) {sensorToMechGearRatio = value;}

  public boolean isBottomExtended() {return bottomExtended;}
  public boolean isTopExtended() {return topExtended;}

  public void setBottomExtended(boolean value) {bottomExtended = value;}
  public void setTopExtended(boolean value) {topExtended = value;}

  public boolean getLimitSwitch() {return limitSwitch.get();}

  public boolean isZeroed() {return isZeroed;}
  public void setZeroed(boolean value) {isZeroed = value;}

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Climber");

    builder.addBooleanProperty("Bottom Hooks Extended", this::isBottomExtended, this::setBottomExtended);
    builder.addBooleanProperty("Top Hooks Extended", this::isTopExtended, this::setTopExtended);

    builder.addBooleanProperty("is Zeroed", this::isZeroed, this::setZeroed);
    builder.addBooleanProperty("Limit Reached", this::getLimitSwitch, null);

    builder.addDoubleProperty("Velocity (rot/s)", this::getVelocity, null);
    builder.addDoubleProperty("Acceleration (rot/s^2)", this::getAcceleration, null);

    builder.addDoubleProperty("Target Acceleration (rot/s^2)", this::getTargetAcceleration, this::setTargetAcceleration);
    builder.addDoubleProperty("Target Velocity (rot/s)", this::getTargetVelocity, this::setTargetVelocity);

    builder.addDoubleProperty("Profile Velocity (rot/s)", this::getProfileVelocity, null);
    builder.addDoubleProperty("Profile Acceleration (rot/s)", this::getProfileAcceleration, null);

    builder.addDoubleProperty("Stator Current (A)", this::getStatorCurrent, null);

    builder.addDoubleProperty("Sensor to Mech Gear Ratio", this::getGearRatio, this::setGearRatio);

    builder.addDoubleProperty("Empty kG", this::getSlot0_kG, this::setSlot0_kG);
    builder.addDoubleProperty("Empty kV", this::getSlot0_kV, this::setSlot0_kV);
    builder.addDoubleProperty("Empty kA", this::getSlot0_kA, this::setSlot0_kA);
    builder.addDoubleProperty("Empty kP", this::getSlot0_kP, this::setSlot0_kP);
    builder.addDoubleProperty("Empty kI", this::getSlot0_kI, this::setSlot0_kI);
    builder.addDoubleProperty("Empty kD", this::getSlot0_kI, this::setSlot0_kI);

    builder.addDoubleProperty("Full kG", this::getSlot1_kG, this::setSlot1_kG);
    builder.addDoubleProperty("Full kV", this::getSlot1_kV, this::setSlot1_kV);
    builder.addDoubleProperty("Full kA", this::getSlot1_kA, this::setSlot1_kA);
    builder.addDoubleProperty("Full kP", this::getSlot1_kP, this::setSlot1_kP);
    builder.addDoubleProperty("Full kI", this::getSlot1_kI, this::setSlot1_kI);
    builder.addDoubleProperty("Full kD", this::getSlot1_kD, this::setSlot1_kD);
  }

  @Override
  public void periodic() {
    if(!isZeroed && getLimitSwitch()) {
      elevator.setPosition(0);
      setZeroed(true);
    }
  }
}
