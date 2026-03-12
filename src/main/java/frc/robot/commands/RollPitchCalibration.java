package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

import org.apache.commons.math3.stat.regression.OLSMultipleLinearRegression;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class RollPitchCalibration extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight limelight;

    private ArrayList<double[]> input = new ArrayList<>();
    private String limelightName = "limelight-left";
    private double fiducialID = 19;

    public RollPitchCalibration(CommandSwerveDrivetrain drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
    }

    @Override
    public void initialize() {
        input.clear();
    }

    @Override
    public void execute() {
      Pose3d pose = limelight.getTargetPose3d_CameraSpace(limelightName);
      if (!(Math.abs(pose.getX()) < 0.01 && Math.abs(pose.getY()) < 0.01 && Math.abs(pose.getZ()) < 0.01)) {
        double[] point = {pose.getX(), pose.getY(), pose.getZ()};
        input.add(point);
        System.out.println("Measurement taken. Total: " + input.size());
      }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (input.isEmpty()) {
            System.out.println("Calibration failed: No samples.");
            return;
        }

        double[][] X = new double[input.size()][3];
        double[] Z = new double[input.size()];
        for (int i = 0; i < input.size(); i++) {
            double[] p = input.get(i);
            X[i][0] = p[0];
            X[i][1] = p[1];
            X[i][2] = 1.0;
            Z[i] = p[2]; 
        }

        OLSMultipleLinearRegression regression = new OLSMultipleLinearRegression();
        regression.newSampleData(Z, X);

        double[] params = regression.estimateRegressionParameters();
        double a = params[0];
        double b = params[1];
        double d = params[2];

        double[] normal = new double[]{a, b, -1};
        double mag = Math.sqrt(a*a + b*b + 1);
        normal[0] /= mag;
        normal[1] /= mag;
        normal[2] /= mag;

        double roll = Math.atan2(normal[1], normal[2]);
        double pitch = -Math.atan2(normal[0], normal[2]);

        System.out.println("RESULT");
        System.out.println("Roll = " +  roll + "\n Pitch = " + pitch);
    }
}