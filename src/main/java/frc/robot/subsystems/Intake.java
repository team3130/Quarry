// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX intake;
  private final TalonFX pivot;

  private final DigitalInput limitSwitch;
  private boolean isZeroed = false;
  private boolean isIntaking = false;
  
  private double intakeSpeed = 0.9;

  private final MotionMagicVoltage voltRequest;
  private final TalonFXConfiguration motorConfig;

  private final Slot0Configs config;
  private double kG = 0;
  private double kV = 0;
  private double kA = 0;
  private double kP = 48;
  private double kI = 0;
  private double kD = 0;

  private final MotionMagicVelocityVoltage voltRequestBars;
  private final TalonFXConfiguration motorConfigBars;

  private final Slot0Configs configBars;
  private double kVBars = 0.2;
  private double kABars = 0.002;
  private double kPBars = 0.75;
  private double kIBars = 0;
  private double kDBars = 0;

  private double sensorToMechGearRatioBars = 2;

  private double targetAccelerationBars = 200;
  private double targetVelocityBars = 40;

  private double sensorToMechGearRatio = 200;
  private double offset = 0;

  private double targetAcceleration = 32;
  private double targetVelocity = 2;

  private double outPos = 0.22;
  private double inPos = 0;
  private double halfPos = 0.05;
  public Intake() {
    limitSwitch = new DigitalInput(Constants.IDs.intakeLimit);

    intake = new TalonFX(Constants.CAN.intake);
    intake.getConfigurator().apply(new TalonFXConfiguration()
    .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive)));

    configBars = new Slot0Configs();
    configBars.kV = kVBars;
    configBars.kA = kABars;
    configBars.kP = kPBars;
    configBars.kI = kIBars;
    configBars.kD = kDBars;

    motorConfigBars = new TalonFXConfiguration();
    motorConfigBars.MotorOutput = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);
    motorConfigBars.MotionMagic = new MotionMagicConfigs()
        .withMotionMagicAcceleration(targetAccelerationBars)
        .withMotionMagicCruiseVelocity(targetVelocityBars);
    motorConfigBars.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(sensorToMechGearRatioBars);
    motorConfigBars.Slot0 = configBars;

    intake.getConfigurator().apply(motorConfigBars);

    voltRequestBars = new MotionMagicVelocityVoltage(0);

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

    pivot.setPosition(0);
  }

  public void extendIntakeToSetpoint(double setpoint) {
    pivot.setControl(voltRequest.withPosition(setpoint));
  }

  public void intakeOut() {
    if(isZeroed()) {
      pivot.setControl(voltRequest.withPosition(outPos));
    } else {
      basicPivotUp();
    }
  }
  public void intakeIn() {
    pivot.setControl(voltRequest.withPosition(inPos));
  }
  public void intakeHalf() {
    pivot.setControl(voltRequest.withPosition(halfPos));
  }
  public void intakePivotToSetpoint(double pos) {
    pivot.setControl(voltRequest.withPosition(pos));
  }

  public void basicPivotUp() {pivot.set(-0.2);}
  public void basicPivotDown() {pivot.set(0.2);}
  public void stopPivot() {pivot.set(0);}

  public void runIntakeBasic() {intake.set(intakeSpeed);}
  public void reverseIntakeBasic() {intake.set(-intakeSpeed);}
  public void stopIntake() {intake.set(0);}

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

  public double getPosition() {return pivot.getPosition().getValueAsDouble();}
  public double getVelocity() {return pivot.getVelocity().getValueAsDouble();}
  public double getAcceleration() {return pivot.getAcceleration().getValueAsDouble();}

  public double getTargetAcceleration() {return targetAcceleration;}
  public void setTargetAcceleration(double value) {targetAcceleration = value;}

  public double getTargetVelocity() {return targetVelocity;}
  public void setTargetVelocity(double value) {targetVelocity = value;}

  public double getProfilePosition() {
    return pivot.getClosedLoopReference().getValueAsDouble();
  }
  public double getProfileVelocity() {
    return pivot.getClosedLoopReferenceSlope().getValueAsDouble();
  }

  public double getStatorCurrent() {return pivot.getStatorCurrent().getValueAsDouble();}

  public double getGearRatio() {return sensorToMechGearRatio;}
  public void setGearRatio(double value) {sensorToMechGearRatio = value;}

  public double getOutPos() {return outPos;}
  public double getInPos() {return inPos;}
  public void setOutPos(double value) {outPos = value;}
  public void setInPos(double value) {inPos = value;}

  public void runIntakeAtVelocity(double newVelocity) {
    intake.setControl(voltRequestBars.withVelocity(newVelocity));
  }
  public void runIntake() {
    intake.setControl(voltRequestBars.withVelocity(targetVelocityBars));
  }

  public double getkVBars() {return kVBars;}
  public double getkABars() {return kABars;}
  public double getkPBars() {return kPBars;}
  public double getkIBars() {return kIBars;}
  public double getkDBars() {return kDBars;}
  public void setkVBars(double value) {kVBars = value;}
  public void setkABars(double value) {kABars = value;}
  public void setkPBars(double value) {kPBars = value;}
  public void setkIBars(double value) {kIBars = value;}
  public void setkDBars(double value) {kDBars = value;}

  public void updatePIDBars() {
    configBars.kV = kVBars;
    configBars.kA = kABars;
    configBars.kP = kPBars;
    configBars.kI = kIBars;
    configBars.kD = kDBars;
    motorConfigBars.Slot0 = configBars;
    intake.getConfigurator().apply(motorConfigBars);
  }

  public double getVelocityBars() {return intake.getVelocity().getValueAsDouble();}
  public double getAccelerationBars() {return intake.getAcceleration().getValueAsDouble();}

  public double getTargetAccelerationBars() {return targetAccelerationBars;}
  public void setTargetAccelerationBars(double value) {targetAccelerationBars = value;}

  public double getTargetVelocityBars() {return targetVelocityBars;}
  public void setTargetVelocityBars(double value) {targetVelocityBars = value;}

  public double getProfileVelocityBars() {
    return intake.getClosedLoopReference().getValueAsDouble();
  }
  public double getProfileAccelerationBars() {
    return intake.getClosedLoopReferenceSlope().getValueAsDouble();
  }

  public double getGearRatioBars() {return sensorToMechGearRatioBars;}
  public void setGearRatioBars(double value) {sensorToMechGearRatioBars = value;}

  public double getStatorCurrentBars() {return intake.getStatorCurrent().getValueAsDouble();}

  public boolean atLimit() {return limitSwitch.get();}

  public boolean isZeroed() {return isZeroed;}
  public void setZeroed(boolean value) {isZeroed = value;}

  public boolean getIsIntaking() {return isIntaking;}
  public void setIsIntaking(boolean value) {isIntaking = value;}

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Intake");

    builder.addBooleanProperty("is Zeroed", this::isZeroed, this::setZeroed);
    builder.addBooleanProperty("Limit Reached", this::atLimit, null);
    builder.addBooleanProperty("Is Intaking", this::getIsIntaking, this::setIsIntaking);

    builder.addDoubleProperty("Position (rot)", this::getPosition, null);
    builder.addDoubleProperty("Velocity (rot/s)", this::getVelocity, null);
    builder.addDoubleProperty("Acceleration (rot/s^2)", this::getAcceleration, null);

    builder.addDoubleProperty("Target Acceleration (rot/s^2)", this::getTargetAcceleration, this::setTargetAcceleration);
    builder.addDoubleProperty("Target Velocity (rot/s)", this::getTargetVelocity, this::setTargetVelocity);

    builder.addDoubleProperty("Profile Velocity (rps)", this::getProfileVelocity, null);
    builder.addDoubleProperty("Profile Position (rot)", this::getProfilePosition, null);

    builder.addDoubleProperty("Stator Current (A)", this::getStatorCurrent, null);

    builder.addDoubleProperty("Sensor to Mech Gear Ratio", this::getGearRatio, this::setGearRatio);

    builder.addDoubleProperty("kV", this::getkV, this::setkV);
    builder.addDoubleProperty("kA", this::getkA, this::setkA);
    builder.addDoubleProperty("kP", this::getkP, this::setkP);
    builder.addDoubleProperty("kI", this::getkI, this::setkI);
    builder.addDoubleProperty("kD", this::getkD, this::setkD);

    builder.addDoubleProperty("Intake Velocity (rps)", this::getVelocityBars, null);
    builder.addDoubleProperty("Intake Acceleration (rps^2)", this::getAccelerationBars, null);

    builder.addDoubleProperty("Intake Target Acceleration (rps^2)", this::getTargetAccelerationBars, this::setTargetAccelerationBars);
    builder.addDoubleProperty("Intake Target Velocity (rps)", this::getTargetVelocityBars, this::setTargetVelocityBars);

    builder.addDoubleProperty("Intake Profile Velocity (rps)", this::getProfileVelocityBars, null);
    builder.addDoubleProperty("Intake Profile Acceleration (rps^2)", this::getProfileAccelerationBars, null);

    builder.addDoubleProperty("Intake Stator Current (A)", this::getStatorCurrentBars, null);

    builder.addDoubleProperty("Intake Sensor to Mech Gear Ratio", this::getGearRatioBars, this::setGearRatioBars);

    builder.addDoubleProperty("Intake kV", this::getkVBars, this::setkVBars);
    builder.addDoubleProperty("Intake kA", this::getkABars, this::setkABars);
    builder.addDoubleProperty("Intake kP", this::getkPBars, this::setkPBars);
    builder.addDoubleProperty("Intake kI", this::getkIBars, this::setkIBars);
    builder.addDoubleProperty("Intake kD", this::getkDBars, this::setkDBars);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(atLimit() && !isZeroed) {
      pivot.setPosition(0);
      setZeroed(true);
    }
  }
}
