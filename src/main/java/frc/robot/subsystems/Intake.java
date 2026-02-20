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

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX intake;
  private final TalonFX pivot;

  private final DigitalInput limitSwitch;
  private boolean isZeroed = false;
  
  private double intakeSpeed = 1;

  private final MotionMagicVoltage voltRequest;
  private final TalonFXConfiguration motorConfig;

  private final Slot0Configs config;
  private double kG = 0;
  private double kV = 0.12;
  private double kA = 0;
  private double kP = 0;
  private double kI = 0;
  private double kD = 0;

  private double sensorToMechGearRatio = 1;
  private double offset = 0;

  private double targetAcceleration = 2;
  private double targetVelocity = 2;

  private double outPos = 0;
  private double inPos = 0.25;
  public Intake() {
    limitSwitch = new DigitalInput(Constants.IDs.intakeLimit);

    intake = new TalonFX(Constants.CAN.intake);
    intake.getConfigurator().apply(new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive)));

    pivot = new TalonFX(Constants.CAN.intakePivot);

    config = new Slot0Configs();
    config.kG = kG;
    config.kV = kV;
    config.kA = kA;
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    config.GravityType = GravityTypeValue.Arm_Cosine;
    config.GravityArmPositionOffset = offset;


    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);
    motorConfig.MotionMagic = new MotionMagicConfigs()
        .withMotionMagicAcceleration(targetAcceleration)
        .withMotionMagicCruiseVelocity(targetVelocity);
    motorConfig.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(sensorToMechGearRatio);
    motorConfig.Slot0 = config;

    pivot.getConfigurator().apply(motorConfig);

    voltRequest = new MotionMagicVoltage(0);
  }

  public void extendIntakeToSetpoint(double setpoint) {
    pivot.setControl(voltRequest.withPosition(setpoint));
  }
  public void intakeOut() {
    pivot.setControl(voltRequest.withPosition(outPos));
  }
  public void intakeIn() {
    pivot.setControl(voltRequest.withPosition(inPos));
  }

  public void runIntake() {
    intake.set(intakeSpeed);
  }
  public void reverseIntake() {
    intake.set(-intakeSpeed);
  }
  public void stopIntake() {
    intake.set(0);
  }

  public double getkG() {return kG;}
  public double getkV() {return kV;}
  public double getkA() {return kA;}
  public double getkP() {return kP;}
  public double getkI() {return kI;}
  public double getkD() {return kD;}
  public void setkG(double value) {kG = value;}
  public void setkV(double value) {kV = value;}
  public void setkA(double value) {kA = value;}
  public void setkP(double value) {kP = value;}
  public void setkI(double value) {kI = value;}
  public void setkD(double value) {kD = value;}

  public void updatePID() {
    config.kG = kG;
    config.kV = kV;
    config.kA = kA;
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    motorConfig.Slot0 = config;
    pivot.getConfigurator().apply(motorConfig);
  }

  public double getVelocity() {return pivot.getVelocity().getValueAsDouble();}
  public double getAcceleration() {return pivot.getAcceleration().getValueAsDouble();}

  public double getTargetAcceleration() {return targetAcceleration;}
  public void setTargetAcceleration(double value) {targetAcceleration = value;}

  public double getTargetVelocity() {return targetVelocity;}
  public void setTargetVelocity(double value) {targetVelocity = value;}

  public double getProfileVelocity() {
    return pivot.getClosedLoopReference().getValueAsDouble();
  }
  public double getProfileAcceleration() {
    return pivot.getClosedLoopReferenceSlope().getValueAsDouble();
  }

  public double getStatorCurrent() {return pivot.getStatorCurrent().getValueAsDouble();}

  public double getGearRatio() {return sensorToMechGearRatio;}
  public void setGearRatio(double value) {sensorToMechGearRatio = value;}

  public double getOutPos() {return outPos;}
  public double getInPos() {return inPos;}
  public void setOutPos(double value) {outPos = value;}
  public void setInPos(double value) {inPos = value;}

  public boolean atLimit() {return limitSwitch.get();}

  public boolean isZeroed() {return isZeroed;}
  public void setZeroed(boolean value) {isZeroed = value;}

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Intake");

    builder.addBooleanProperty("is Zeroed", this::isZeroed, this::setZeroed);
    builder.addBooleanProperty("Limit Reached", this::atLimit, null);

    builder.addDoubleProperty("Velocity (rot/s)", this::getVelocity, null);
    builder.addDoubleProperty("Acceleration (rot/s^2)", this::getAcceleration, null);

    builder.addDoubleProperty("Target Acceleration (rot/s^2)", this::getTargetAcceleration, this::setTargetAcceleration);
    builder.addDoubleProperty("Target Velocity (rot/s)", this::getTargetVelocity, this::setTargetVelocity);

    builder.addDoubleProperty("Profile Velocity (rot/s)", this::getProfileVelocity, null);
    builder.addDoubleProperty("Profile Acceleration (rot/s)", this::getProfileAcceleration, null);

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
    if(atLimit() && !isZeroed()) {
      pivot.setPosition(0);
      setZeroed(true);
    }
  }
}
