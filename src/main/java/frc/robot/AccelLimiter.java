package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AccelLimiter {
    private boolean isAngleReal = false;
    public final SlewRateLimiter driveLimiter;
    public final SlewRateLimiter thetaLimiter;
    public final PowerAccount translationAccount;
    public final PowerAccount rotationAccount;

    public AccelLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue, double rateLimit) {
        driveLimiter = new SlewRateLimiter(positiveRateLimit, negativeRateLimit, initialValue, 60.0, 15);
        thetaLimiter = new SlewRateLimiter(rateLimit, 1000);
        translationAccount = PowerBank.getInstance().openAccount("Translation", 100);
        rotationAccount = PowerBank.getInstance().openAccount("Rotation", 100);
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
                    translationAccount.setMaxRequest(driveLimiter.getPowerFromAcceleration(driveLimiter.getMaxAcceleration(), driveLimiter.lastValue()));
                    driveLimiter.setPositiveRateLimit(driveLimiter.getAccelerationFromPower(translationAccount.getAllowance(), driveLimiter.lastValue()));
                    mag = driveLimiter.calculate(mag);
                    double thetaLimiterConstant = 10;
                    double limit = thetaLimiterConstant /mag;
                    thetaLimiter.updateValues(limit, -limit);
                    var theta = thetaLimiter.angleCalculate(vector.getAngle().getRadians());
                    Translation2d newVector = new Translation2d(mag, new Rotation2d(theta));
                    return new ChassisSpeeds(newVector.getX(), newVector.getY(), rotation);
                }
            }
            //here we continue if we are decelerating, either small mag or big turn.
            thetaLimiter.reset(thetaLimiter.lastValue());
            translationAccount.setMaxRequest(driveLimiter.getPowerFromAcceleration(driveLimiter.getMaxAcceleration(), driveLimiter.lastValue()));
            driveLimiter.setPositiveRateLimit(driveLimiter.getAccelerationFromPower(translationAccount.getAllowance(), driveLimiter.lastValue()));
            var newMag = driveLimiter.calculate(0);
            Rotation2d angle = new Rotation2d(thetaLimiter.lastValue());
            Translation2d newVector = new Translation2d(newMag, angle);
            if(newMag < 0.001){ // we have stopped moving
                isAngleReal = false;
            }
            return new ChassisSpeeds(newVector.getX(), newVector.getY(), rotation);
        }
        else { //if angle is not real, then we were standing still 20 ms ago
            if(vector.getNorm() < 0.001){ //if the norm is still tiny, then keep idling
                translationAccount.setMaxRequest(0);
                driveLimiter.reset(0);
                return new ChassisSpeeds(0,0, rotation);
            }
            else { //if the norm is significant, start driving
                isAngleReal = true;
                thetaLimiter.reset(vector.getAngle().getRadians());
                translationAccount.setMaxRequest(driveLimiter.getPowerFromAcceleration(driveLimiter.getMaxAcceleration(), driveLimiter.lastValue()));
                driveLimiter.setPositiveRateLimit(driveLimiter.getAccelerationFromPower(translationAccount.getAllowance(), driveLimiter.lastValue()));
                var mag = driveLimiter.calculate(vector.getNorm());
                Translation2d newVector = new Translation2d(mag, vector.getAngle());
                return new ChassisSpeeds(newVector.getX(), newVector.getY(), rotation);
            }
        }
    }
}
