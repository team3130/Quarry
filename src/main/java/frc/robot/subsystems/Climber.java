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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private final Solenoid bottomHooks;
  private final Solenoid topHooks;

  private final TalonFX elevator;

  private final MotionMagicVoltage voltRequest;
  private final TalonFXConfiguration motorConfig;

  private final Slot0Configs config;
  private double kV = 0.12;
  private double kA = 0;
  private double kP = 0;
  private double kI = 0;
  private double kD = 0;

  private double reachPos = 0;
  private double bottomPos = 0;

  private double sensorToMechGearRatio = 1;

  private double targetAcceleration = 2;
  private double targetVelocity = 2;

  private final DigitalInput limitSwitch;
  private boolean isZeroed = false;

  private boolean bottomExtended = false;
  private boolean topExtended = false;
  /** Creates a new Climber. */
  public Climber() {
    elevator = new TalonFX(Constants.CAN.climberElevator);

    limitSwitch = new DigitalInput(Constants.IDs.climberLimit);

    bottomHooks = new Solenoid(35, PneumaticsModuleType.CTREPCM, Constants.IDs.bottomHooks);
    topHooks = new Solenoid(Constants.CAN.PCM, PneumaticsModuleType.CTREPCM, Constants.IDs.topHooks);

    config = new Slot0Configs();
    config.kV = kV;
    config.kA = kA;
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;

    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);
    motorConfig.MotionMagic = new MotionMagicConfigs()
        .withMotionMagicAcceleration(targetAcceleration)
        .withMotionMagicCruiseVelocity(targetVelocity);
    motorConfig.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(sensorToMechGearRatio);
    motorConfig.Slot0 = config;

    elevator.getConfigurator().apply(motorConfig);

    voltRequest = new MotionMagicVoltage(0);
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


  public void extendBottomHooks() {bottomHooks.set(true);}
  public void retractBottomHooks() {bottomHooks.set(false);}

  public void extendTopHooks() {topHooks.set(true);}
  public void retractTopHooks() {topHooks.set(false);}

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

    builder.addDoubleProperty("kV", this::getkV, this::setkV);
    builder.addDoubleProperty("kA", this::getkA, this::setkA);
    builder.addDoubleProperty("kP", this::getkP, this::setkP);
    builder.addDoubleProperty("kI", this::getkI, this::setkI);
    builder.addDoubleProperty("kD", this::getkD, this::setkD);
  }

  @Override
  public void periodic() {
    if(!isZeroed && getLimitSwitch()) {
      elevator.setPosition(0);
      setZeroed(true);
    }
  }
}
