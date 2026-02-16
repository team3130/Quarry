package frc.robot;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class MySlewRateLimiter implements Sendable {
    private double positiveRateLimit;
    private double negativeRateLimit;
    private double prevVal;
    private double prevTime;
    private final double posRateLimitSlope = -(Constants.Swerve.maxAccelerationFromRest / TunerConstants.kSpeedAt12Volts.magnitude());
    private final double posRateLimitYIntersect = Constants.Swerve.maxAccelerationFromRest;

    public double getLinearPositiveRateLimit(double norm){
        return norm * posRateLimitSlope + posRateLimitYIntersect;
    }

    public MySlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue) {
        this.positiveRateLimit = positiveRateLimit;
        this.negativeRateLimit = negativeRateLimit;
        this.prevVal = initialValue;
        this.prevTime = MathSharedStore.getTimestamp();
    }

    public MySlewRateLimiter(double rateLimit) {
        this(rateLimit, -rateLimit, (double)0.0F);
    }

    public double calculate(double input) {
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - this.prevTime;
        double delta = input - this.prevVal;
        this.prevVal += MathUtil.clamp(delta, this.negativeRateLimit * elapsedTime, this.positiveRateLimit * elapsedTime);
        this.prevTime = currentTime;
        return this.prevVal;
    }

    public double angleCalculate(double input) {
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - this.prevTime;
        this.prevVal += MathUtil.clamp(getDelta(input), this.negativeRateLimit * elapsedTime, this.positiveRateLimit * elapsedTime);
        this.prevTime = currentTime;
        this.prevVal = MathUtil.angleModulus(this.prevVal);
        return this.prevVal;
    }

    public double lastValue() {
        return this.prevVal;
    }

    public void reset(double value) {
        this.prevVal = value;
        this.prevTime = MathSharedStore.getTimestamp();
    }

    public void updateValues(double positiveRateLimit, double negativeRateLimit) {
        this.positiveRateLimit = positiveRateLimit;
        this.negativeRateLimit = negativeRateLimit;
    }

    public void setPositiveRateLimit(double positiveRateLimit) {
        this.positiveRateLimit = positiveRateLimit;
    }
    public double getPositiveRateLimit() {return positiveRateLimit;}

    public double getElapsedTime() {
        return MathSharedStore.getTimestamp() - prevTime;
    }

    public double getDelta(double input) {
        double delta = input - this.prevVal;
        if(delta > Math.PI) {
            delta -= Math.PI * 2;
        } else if(delta < -Math.PI) {
            delta += Math.PI * 2;
        }
        return delta;
    }

    public Translation2d wrapAngle(Translation2d targetVector, CommandSwerveDrivetrain driveTrain) {
        Translation2d currentVector =
                new Translation2d(driveTrain.getKinematics().toChassisSpeeds(driveTrain.getState().ModuleStates).vxMetersPerSecond,
                        driveTrain.getKinematics().toChassisSpeeds(driveTrain.getState().ModuleStates).vxMetersPerSecond);
        Rotation2d delta = targetVector.getAngle().minus(currentVector.getAngle());
        return Math.abs(delta.getDegrees()) > 90.0 ?
                new Translation2d(-targetVector.getNorm(), targetVector.getAngle().rotateBy(Rotation2d.kPi))
                : new Translation2d(targetVector.getNorm(), targetVector.getAngle());
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Slew Rate Limiter");

        builder.addDoubleProperty("Positive Rate Limit", this::getPositiveRateLimit, null);
        builder.addDoubleProperty("Last Value", this::lastValue, null);
    }
}
