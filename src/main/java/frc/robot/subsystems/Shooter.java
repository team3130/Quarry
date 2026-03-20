// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final TalonFX leftShooter;
  private final TalonFX rightShooter;

  private final MotionMagicVelocityVoltage voltRequest;
  private final TalonFXConfiguration motorConfig;

  private final Slot0Configs config;
  private double kV = 0.11636;
  private double kA = 0.011846;
  private double kP = 0.500000;
  private double kI = 0;
  private double kD = 0;

  private double sensorToMechGearRatio = 1;

  private double accelerationMetersPerSecSquared = 45;
  private final double accelerationRotations = Units.radiansToRotations(accelerationMetersPerSecSquared/Units.inchesToMeters(2));

  private double targetVelocityMetersPerSec = 16;
  private final double targetVelocityRotations = Units.radiansToRotations(targetVelocityMetersPerSec/Units.inchesToMeters(2));

  private double speed = 0.7;

  //SysID
  private final VoltageOut m_voltReq = new VoltageOut(0.0);
  private final SysIdRoutine m_sysIdRoutine;


  //Shooter Curves
    private final CommandSwerveDrivetrain driveTrain;
    private final ShooterHood shooterHood;
    private final double radius = Units.inchesToMeters(2);


    //New Measurment Arrays
    private static final double[] distances = {1.2, 1.5, 2, 2.5, 3.4, 3.8, 4.4};                      //meters
    private static final double[] velocities = {13, 13.48, 13.93, 14.23, 15.8, 16.5, 16.9};    //meters per seconds

    private final double[] linearizeVel = {velocityLinearizer(velocities[0]), velocityLinearizer(velocities[1]),
       velocityLinearizer(velocities[2]), velocityLinearizer(velocities[3])};

    //Interpolation Objects
    InterpolatingDoubleTreeMap tableVel = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap tableVelLin = new InterpolatingDoubleTreeMap();

  /** Creates a new Shooter. */
  public Shooter(CommandSwerveDrivetrain drivetrain, ShooterHood shooterHood) {
    this.driveTrain = drivetrain;
    this.shooterHood = shooterHood;
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

    //SysID
    m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
         null,        // Use default ramp rate (1 V/s)
         Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
         null,        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("shooter_state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> rightShooter.setControl(m_voltReq.withOutput(volts.in(Volts))),
         null,
         this
      )
    );
    //Interpolation Double tree for Velocities
    tableVel.put(distances[0], velocities[0]);
    tableVel.put(distances[1], velocities[1]);
    tableVel.put(distances[2], velocities[2]);
    tableVel.put(distances[3], velocities[3]);
    tableVel.put(distances[4], velocities[4]);
    tableVel.put(distances[5], velocities[5]);
    tableVel.put(distances[6], velocities[6]);

    //Linearized Velocity Table
    tableVelLin.put(distances[0], linearizeVel[0]);
    tableVelLin.put(distances[1], linearizeVel[1]);
    tableVelLin.put(distances[2], linearizeVel[2]);
    tableVelLin.put(distances[3], linearizeVel[3]);
  }

  //SysID
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public void revAtVelocity(double velocityMetersPerSec) {
    Translation2d ballVelocityVector = driveTrain.getTranslationToHub();
    ballVelocityVector = ballVelocityVector.times(0.45 * velocityMetersPerSec/ballVelocityVector.getNorm());
    ballVelocityVector = ballVelocityVector.times(Math.cos(Math.toRadians(360 * shooterHood.autoAimValue() + 9)));
    Translation2d robotVelocityVector = new Translation2d(driveTrain.getRobotRelativeSpeeds().vxMetersPerSecond, driveTrain.getRobotRelativeSpeeds().vyMetersPerSecond);
    Translation2d velocityVector = ballVelocityVector.minus(robotVelocityVector);
    double newVelocityMetersPerSec = velocityVector.getNorm();

    double radsPerSec = newVelocityMetersPerSec / Units.inchesToMeters(2);
    double rotsPerSec = Units.radiansToRotations(radsPerSec);
    rightShooter.setControl(voltRequest.withVelocity(rotsPerSec));
  }

  public void autoRev() {
    rightShooter.setControl(voltRequest.withVelocity(interpolTargetSpeed()));
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
    driveTrain.setIsShooting(false);
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

  public double velocityLinearizer(double speed) {return speed*speed;}

  public double getInterPolVel() {
    double velmps = tableVelLin.get(driveTrain.getDistanceFromHub());//Change tableVel to tableVelLin for linearized velocity.
    setTargetVelocity(velmps);
    return Math.sqrt(velmps);
  }

    // Interpolation Request for Velocity
    public double interpolTargetSpeed() {
      double distance = driveTrain.getDistanceFromHub();
      double velmps = tableVel.get(distance);//Change tableVel to tableVelLin for linearized velocity.
      Translation2d hubVector = driveTrain.getTranslationToHub();
      hubVector = new Translation2d(1, 0);
      Translation2d ballVelocityVector = hubVector.times(0.5 * velmps/hubVector.getNorm());
      Translation2d ballHorizontalVelocityVector = ballVelocityVector.times(Math.cos(Math.toRadians(81 - 360 * shooterHood.autoAimValue())));
      Translation2d ballVerticalVelocityVector = ballVelocityVector.times(Math.sin(Math.toRadians(81 - 360 * shooterHood.autoAimValue())));
      Translation2d robotVelocityVector = new Translation2d(driveTrain.getFieldRelativeSpeeds().vxMetersPerSecond, driveTrain.getFieldRelativeSpeeds().vyMetersPerSecond);
      robotVelocityVector = new Translation2d(0, 0);
      Translation2d velocityVector = ballHorizontalVelocityVector.minus(robotVelocityVector);
      System.out.println(velocityVector.getX() + " " + velocityVector.getY());
      double velocityMetersPerSec = 2 * (Math.hypot(velocityVector.getNorm(), ballVerticalVelocityVector.getNorm()));
      System.out.println(velocityMetersPerSec);
      driveTrain.setAngleSetpoint(velocityVector.getAngle().getDegrees());
      System.out.println(velocityVector.getAngle().getDegrees());
      double angleDegrees = Math.atan(ballVerticalVelocityVector.getNorm()/velocityVector.getNorm());
      System.out.println(angleDegrees);
      shooterHood.setAutoAimValue(Math.max((81 - angleDegrees)/360, 0.0001));
      driveTrain.setIsShooting(true);

      setTargetVelocity(velocityMetersPerSec);
      double radspersec = velocityMetersPerSec/(radius);
      double rotspersec = Units.radiansToRotations(radspersec);
      return rotspersec;
    }

  public double getStatorCurrent() {return rightShooter.getStatorCurrent().getValueAsDouble();}

  public double getGearRatio() {return sensorToMechGearRatio;}
  public void setGearRatio(double value) {sensorToMechGearRatio = value;}

  public boolean isAtVelocity() {return Math.abs(getVelocity() - getTargetVelocity()) < 0.1;}

  public double getDistanceToHub() {
    return driveTrain.getDistanceFromHub();
  }

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

    //builder.addDoubleProperty("Distance to Hub", this::getDistanceToHub, null);

    builder.addDoubleProperty("kV", this::getkV, this::setkV);
    builder.addDoubleProperty("kA", this::getkA, this::setkA);
    builder.addDoubleProperty("kP", this::getkP, this::setkP);
    builder.addDoubleProperty("kI", this::getkI, this::setkI);
    builder.addDoubleProperty("kD", this::getkD, this::setkD);
  }

  @Override
  public void periodic() {
    System.out.println(getDistanceToHub());
    // This method will be called once per scheduler run
    //updatePID();
  }
}
