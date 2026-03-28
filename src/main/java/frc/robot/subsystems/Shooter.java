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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.PowerAccount;
import frc.robot.PowerBank;
import frc.robot.SlewRateLimiter;

public class Shooter extends SubsystemBase {
  private boolean isShooting = false;

  private final TalonFX leftShooter;
  private final TalonFX rightShooter;

  private final MotionMagicVelocityVoltage voltRequest;
  private final TalonFXConfiguration motorConfig;

  private final SlewRateLimiter slewRateLimiter;
  private final PowerAccount shooterAccount;

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
    private final double radius = Units.inchesToMeters(2);
    private final double frictionCoef = 0.62;

  //New Measurment Arrays
  private static final double[] distances = {1.2, 1.5, 2, 2.5, 3.4, 3.8, 4.4};                      //meters
  private static final double[] velocities = {13, 13.48, 13.93, 14.23, 15.8, 16.5, 16.9};    //meters per seconds

  private final double[] linearizeVel = {velocityLinearizer(velocities[0]), velocityLinearizer(velocities[1]),
      velocityLinearizer(velocities[2]), velocityLinearizer(velocities[3])};

  //Interpolation Objects
  InterpolatingDoubleTreeMap tableVel = new InterpolatingDoubleTreeMap();
  InterpolatingDoubleTreeMap tableVelLin = new InterpolatingDoubleTreeMap();

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

    slewRateLimiter = new SlewRateLimiter(100, -100, 0, 0.00007, 100);
    shooterAccount = PowerBank.getInstance().openAccount("shooter", 1);

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

  public void revAtVelocity(double velocityMetersPerSec, CommandSwerveDrivetrain driveTrain, ShooterHood shooterHood) {
    Translation2d ballVelocityVector = driveTrain.getTranslationToHub();
    ballVelocityVector = ballVelocityVector.times(0.45 * velocityMetersPerSec/ballVelocityVector.getNorm());
    ballVelocityVector = ballVelocityVector.times(Math.cos(Math.toRadians(360 * shooterHood.getTableAutoAimValue(driveTrain.getDistanceFromHub()) + 9)));
    Translation2d robotVelocityVector = new Translation2d(driveTrain.getRobotRelativeSpeeds().vxMetersPerSecond, driveTrain.getRobotRelativeSpeeds().vyMetersPerSecond);
    Translation2d velocityVector = ballVelocityVector.minus(robotVelocityVector);
    double newVelocityMetersPerSec = velocityVector.getNorm();

    double radsPerSec = newVelocityMetersPerSec / Units.inchesToMeters(2);
    double rotsPerSec = Units.radiansToRotations(radsPerSec);
    rightShooter.setControl(voltRequest.withVelocity(rotsPerSec));
  }

  public void autoRev(CommandSwerveDrivetrain drivetrain, ShooterHood shooterHood) {
    rightShooter.setControl(voltRequest.withVelocity(interpolTargetSpeed(drivetrain, shooterHood)));
  }

  public void rev() {
    rightShooter.setControl(voltRequest.withVelocity(targetVelocityRotations));
  }

  public void revWithGivenPower(boolean forward) {
    int sign = forward ? 1 : -1;
    double powerReq = slewRateLimiter.getPowerFromAcceleration(sign*accelerationRotations, slewRateLimiter.lastValue());
    shooterAccount.setMaxRequest(powerReq);
    double rotAccel = slewRateLimiter.getAccelerationFromPower(shooterAccount.getAllowance(), slewRateLimiter.lastValue());
    double newRotVel = slewRateLimiter.calculate(sign*rotAccel);
    rightShooter.setControl(voltRequest.withVelocity(newRotVel/(2 * Math.PI)));
  }

  public void runShooter() {
    rightShooter.set(speed);
  }

  public void reverseShooter() {
    rightShooter.set(-speed);
  }

  public void stopShooter() {
    rightShooter.set(0);
    shooterAccount.setMaxRequest(0);
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

  public double getInterPolVel(double distance) {
    double velmps = tableVelLin.get(distance);//Change tableVel to tableVelLin for linearized velocity.
    setTargetVelocity(velmps);
    return Math.sqrt(velmps);
  }

  // Interpolation Request for Velocity
  public double interpolTargetSpeed(CommandSwerveDrivetrain driveTrain, ShooterHood shooterHood) {
    double distance = driveTrain.getDistanceFromHub();
    double velmps = tableVel.get(distance); 
      
    // 1. Get the direction to the hub (Must point AT the hub)
    Translation2d toHubDir = driveTrain.getTranslationToHub();
      
    // 2. Calculate the "Static" components (what the ball does if robot is still)
    double hoodAngleRads = Math.toRadians(81 - (360 * shooterHood.getTableAutoAimValue(distance)));
    double ballVelHorizontalMag = velmps * Math.cos(hoodAngleRads) * frictionCoef;
    double ballVelVerticalMag = velmps * Math.sin(hoodAngleRads) * frictionCoef;
      
    // 3. Create the Horizontal Ball Velocity Vector (Field Relative)
    Translation2d ballHorizontalVec = new Translation2d(ballVelHorizontalMag, toHubDir.getAngle());

    // 4. Subtract Robot Velocity (Field Relative) - NO MULTIPLIER
    Translation2d robotFieldVel = new Translation2d(
      driveTrain.getSpeeds().vxMetersPerSecond, 
      driveTrain.getSpeeds().vyMetersPerSecond
    ).rotateBy(driveTrain.getStatePose().getRotation());
    // The "Compensated" horizontal vector
    Translation2d compensatedHorizontalVec = ballHorizontalVec.minus(robotFieldVel);

    // 5. UPDATE SETPOINT: This is the angle the ROBOT must face to cancel drift
    driveTrain.setAngleSetpoint(compensatedHorizontalVec.getAngle().getDegrees());

    // 6. Calculate New Total Magnitude (3D hypotenuse)
    double finalTotalVelMps = Math.hypot(compensatedHorizontalVec.getNorm(), ballVelVerticalMag) / frictionCoef;
      
    // 7. Update Hood Angle (The tilt changes because horizontal velocity changed)
    double newHoodAngleDegrees = Math.toDegrees(Math.atan2(ballVelVerticalMag, compensatedHorizontalVec.getNorm()));
    shooterHood.setAutoAimValue((81 - newHoodAngleDegrees) / 360.0);

    setTargetVelocity(finalTotalVelMps);
    //driveTrain.setIsShooting(true);
    return Units.radiansToRotations(finalTotalVelMps / radius);
  }

  public double getStatorCurrent() {return rightShooter.getStatorCurrent().getValueAsDouble();}

  public double getGearRatio() {return sensorToMechGearRatio;}
  public void setGearRatio(double value) {sensorToMechGearRatio = value;}

  public boolean isAtVelocity() {return Math.abs(getVelocity() - getTargetVelocity()) < 0.1;}

  public boolean getIsShooting() {return isShooting;}
  public void setIsShooting(boolean value) {isShooting = value;}

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Shooter");

    builder.addBooleanProperty("Is Shooting", this::getIsShooting, this::setIsShooting);

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
    // This method will be called once per scheduler run
    //updatePID();
  }
}
