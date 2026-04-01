package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants;
import frc.robot.MySlewRateLimiter;
import frc.robot.TunerConstants.TunerSwerveDrivetrain;
import org.json.simple.parser.ParseException;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private PIDController angularPIDController = new PIDController(0.05, 0.01, 0);
    private boolean isFacingHub = false;
    private double angleSetpoint = 0;

    public final MySlewRateLimiter driveLimiter = new MySlewRateLimiter(2, -5, 0);
    public final MySlewRateLimiter thetaLimiter = new MySlewRateLimiter(0);
    private boolean isAngleReal = false;
    private final double deadband = 0.05 * Constants.Swerve.maxSpeed;
    private RobotConfig config;
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }
    private void configureAutoBuilder() {
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
        if (config != null) {
            AutoBuilder.configure(
                    () -> getState().Pose, // Robot pose supplier
                    this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                    () -> getState().Speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds)
                            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                            new PIDConstants( // Translation PID constants
                                    Constants.Swerve.translationPID[0],
                                    Constants.Swerve.translationPID[1],
                                    Constants.Swerve.translationPID[2]),
                            new PIDConstants( // Rotation PID constants
                                    Constants.Swerve.rotationPID[0],
                                    Constants.Swerve.rotationPID[1],
                                    Constants.Swerve.rotationPID[2])
                    ),
                    config, //the robot configuration
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this // Reference to this subsystem to set requirements
            );
        }
    }

    public ChassisSpeeds accelLimitVectorDrive(ChassisSpeeds desiredSpeed) {
        double xAxis = desiredSpeed.vxMetersPerSecond;
        double yAxis = desiredSpeed.vyMetersPerSecond;
        double rotation = desiredSpeed.omegaRadiansPerSecond;
        Translation2d vector = new Translation2d(xAxis, yAxis);
        if(isAngleReal) { //if angle is real, then we were moving 20 ms ago
            if(vector.getNorm() > 0.001){ //if the norm is significant, we continue to move
                double delta = thetaLimiter.getDelta(vector.getAngle().getRadians());
                double cos = Math.cos(delta);
                if(cos > 0){ //positive cos means keep moving (turn angle is small)
                    var mag = vector.getNorm() * cos;
                    driveLimiter.setPositiveRateLimit(driveLimiter.getLinearPositiveRateLimit(driveLimiter.lastValue()));
                    mag = driveLimiter.calculate(mag);
                    double thetaLimiterConstant = 10;
                    double limit = thetaLimiterConstant /mag;
                    thetaLimiter.updateValues(limit, -limit);
                    var theta = thetaLimiter.angleCalculate(vector.getAngle().getRadians());
                    Translation2d newVector = new Translation2d(mag, new Rotation2d(theta));
                    return ChassisSpeeds.discretize(new ChassisSpeeds(newVector.getX(), newVector.getY(), rotation), 0.02);
                }
            }
            //here we continue if we are decelerating, either small mag or big turn.
            thetaLimiter.reset(thetaLimiter.lastValue());
            driveLimiter.setPositiveRateLimit(driveLimiter.getLinearPositiveRateLimit(driveLimiter.lastValue()));
            var newMag = driveLimiter.calculate(0);
            Rotation2d angle = new Rotation2d(thetaLimiter.lastValue());
            Translation2d newVector = new Translation2d(newMag, angle);
            if(newMag < 0.001){ // we have stopped moving
                isAngleReal = false;
            }
            return ChassisSpeeds.discretize(new ChassisSpeeds(newVector.getX(), newVector.getY(), rotation), 0.02);
        }
        else { //if angle is not real, then we were standing still 20 ms ago
            if(vector.getNorm() < 0.001){ //if the norm is still tiny, then keep idling
                driveLimiter.reset(0);
                return ChassisSpeeds.discretize(new ChassisSpeeds(0, 0, rotation), 0.02);
            }
            else { //if the norm is significant, start driving
                isAngleReal = true;
                thetaLimiter.reset(vector.getAngle().getRadians());
                driveLimiter.setPositiveRateLimit(driveLimiter.getLinearPositiveRateLimit(driveLimiter.lastValue()));
                var mag = driveLimiter.calculate(vector.getNorm());
                Translation2d newVector = new Translation2d(mag, vector.getAngle());
                return ChassisSpeeds.discretize(new ChassisSpeeds(newVector.getX(), newVector.getY(), rotation), 0.02);
            }
        }
    }

    public boolean isWithinDeadband(Translation2d vector) {
        return (vector.getNorm() <= deadband);
    }

    public ChassisSpeeds getHIDspeedsMPS(CommandPS5Controller driverController) {
        double xAxis = -driverController.getLeftY();
        double yAxis = -driverController.getLeftX();
        double rotation = -driverController.getRightX();
        xAxis = MathUtil.applyDeadband(xAxis, Constants.Swerve.kDeadband);
        yAxis = MathUtil.applyDeadband(yAxis, Constants.Swerve.kDeadband);
        rotation = MathUtil.applyDeadband(rotation, Constants.Swerve.kDeadband);
        double maxSpeed = 0;
        if(Math.abs(driverController.getRightY()) > 0.7) {
            maxSpeed = 1;
        } else {
            maxSpeed = Constants.Swerve.maxSpeed;
        }
        xAxis *= Math.abs(xAxis) * maxSpeed;
        yAxis *= Math.abs(yAxis) * maxSpeed;
        rotation *= Math.abs(rotation) * Constants.Swerve.maxAngularRate;
        return new ChassisSpeeds(xAxis, yAxis, rotation);
    }

    public double applySingleDeadband(double input, double max) {
        input = MathUtil.applyDeadband(input, 0.04);
        input = input * Math.abs(input);
        return input * max;
    }
    
    public double getAngleSetpoint() {return angleSetpoint;}
    public void setAngleSetpoint(double value) {angleSetpoint = value;}

    public Translation2d getTranslationToHub() {
        if(DriverStation.getAlliance().isEmpty()) {return new Translation2d(0, 0);}
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
          Translation2d originToBlueHub = new Translation2d(Units.inchesToMeters(181.56),Units.inchesToMeters(158.32));
          Translation2d blue = getStatePose().getTranslation().minus(originToBlueHub).unaryMinus();
          return blue;
      } else {
          Translation2d originToRedHub = new Translation2d(Units.inchesToMeters(181.56+287),Units.inchesToMeters(158.32));
          Translation2d red = getStatePose().getTranslation().minus(originToRedHub).unaryMinus();
          return red;
      }
    }

    public Translation2d getTranslationToShuttle() {
        if(DriverStation.getAlliance().isEmpty()) {return new Translation2d(0, 0);}
        if(getStatePose().getY() >= Units.inchesToMeters(158.84)) {//if the robot is above the hub then this is the translation
            if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                Translation2d originToShuttleUpBlue = new Translation2d(Units.inchesToMeters(181.56-39.37),Units.inchesToMeters(158.84+90.95));
                Translation2d blue = getStatePose().getTranslation().minus(originToShuttleUpBlue).unaryMinus();
                return blue;
            } else {
                Translation2d originToShuttleUpRed = new Translation2d(Units.inchesToMeters(181.56+287+39.37),Units.inchesToMeters(158.84+90.95));
                Translation2d red = getStatePose().getTranslation().minus(originToShuttleUpRed).unaryMinus();
                return red;
            }
        } else {//If not above, then below so get below translation
            if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                Translation2d originToShuttleDownBlue = new Translation2d(Units.inchesToMeters(181.56-39.37),Units.inchesToMeters(158.84-90.95));
                Translation2d blue = getStatePose().getTranslation().minus(originToShuttleDownBlue).unaryMinus();
                return blue;
            } else {
                Translation2d originToShuttleDownRed = new Translation2d(Units.inchesToMeters(181.56+287+39.37),Units.inchesToMeters(158.84-90.95));
                Translation2d red = getStatePose().getTranslation().minus(originToShuttleDownRed).unaryMinus();
                return red;
            }
        }
        
    }

    public Translation2d getTranslationForAutoAim() {
        if(getStatePose().getX() > Units.inchesToMeters(182.11) && getStatePose().getX() < Units.inchesToMeters(469.11)) {
        return getTranslationToShuttle();
        } else {
        return getTranslationToHub();
        }
    }

    public double getDistanceForAutoAim() {
    if(getStatePose().getX() > Units.inchesToMeters(182.11) && getStatePose().getX() < Units.inchesToMeters(469.11)) {
      return getTranslationToShuttle().getNorm();
    } else {
      return getTranslationToHub().getNorm();
    }
}


    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param requestSupplier Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    public Pose2d getStatePose() {
        return this.getState().Pose;
    }
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return this.getKinematics().toChassisSpeeds(this.getState().ModuleStates);
    }
    public ChassisSpeeds getSpeeds() {
        return this.getState().Speeds;
    }

    public double getRotationalVelocity(Shooter shooter, Translation2d targetSpotVector, CommandPS5Controller controller) {
        Translation2d robotVector = getState().Pose.getTranslation();
        Translation2d targetVector = targetSpotVector.minus(robotVector);
        double targetAngle = targetVector.getAngle().getDegrees();
        double robotAngle = getState().Pose.getRotation().getDegrees();
        // If shooting correct for movement, otherwise just target hub regardless of where you are
        if (shooter.getIsShooting()) {
            targetAngle = getAngleSetpoint(); 
            driveLimiter.setMaxAccel(3);
            driveLimiter.setNegativeRateLimit(-3);
        } else {
            targetAngle = targetSpotVector.minus(robotVector).getAngle().getDegrees();
        }
        // Keep angles in the range (-180, 180)
        if(targetAngle - robotAngle > 180) {
            targetAngle -= 360;
        } else if(targetAngle - robotAngle < -180) {
            targetAngle += 360;
        }
        // Robot angle is within 3 degrees of target angle and rotational velocityy is less than 0.2 rad/s
        if(Math.abs(robotAngle - targetAngle) < 3 && getState().Speeds.omegaRadiansPerSecond < 1) {
            setFacingHub(true);
        } else {
            setFacingHub(false);
        }
        double angleInput = angularPIDController.calculate(robotAngle, targetAngle);

        Translation2d robotFieldVel = new Translation2d(
            getState().Speeds.vxMetersPerSecond, 
            getState().Speeds.vyMetersPerSecond
        ).rotateBy(getState().Pose.getRotation()); // Field Relative Robot Velocity
        // Calculation of unit tangent vector. Take vector to hub, rotate by 90 degrees to get tangential vector, normalize tangential vector
        Translation2d unitTangent = targetVector.rotateBy(new Rotation2d(Math.PI/2)).div(targetVector.getNorm());
        // Angle correct the opposite direction of movement using w = -v/R
        double angleOutput = -unitTangent.dot(robotFieldVel)/targetVector.getNorm();

        return angleInput + angleOutput;
    }

    public double getRotationalVelocityHub(Shooter shooter, Translation2d hubVector, CommandPS5Controller controller) {
      return getRotationalVelocity(shooter, hubVector, controller);
    }

    public double getRotationalVelocityWhileShuttlingNotHub(Shooter shooter, Translation2d upShuttleVector, Translation2d downShuttleVector, CommandPS5Controller controller) {
        if(getStatePose().getY() >= Units.inchesToMeters(158.84)){
            return getRotationalVelocity(shooter, upShuttleVector, controller);

        } if (getStatePose().getY() < Units.inchesToMeters(158.84)) {//If robot is below the hub on the map
            return getRotationalVelocity(shooter, downShuttleVector, controller);
        } else { //If robot is out of field
            return 0;
        }
    }

    public boolean getFacingHub() {return isFacingHub;}
    public void setFacingHub(boolean value) {isFacingHub = value;}
}
