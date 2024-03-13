package frc.robot.utility;

import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;



public class Vision
{
    public static boolean EvaluateLimelightNew(String limelight)
    {
        double timestamp;
        Pose2d pose;
        double tagarea;
        double tagcount;
        double posediff;
        double xyStdDev = 1;
        double angleStdDev = 12;

        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
        pose = limelightMeasurement.pose;
        posediff = Drive.m_odometry.getEstimatedPosition().getTranslation().getDistance(pose.getTranslation());
        timestamp = limelightMeasurement.timestampSeconds;
        tagcount = limelightMeasurement.tagCount;
        tagarea = limelightMeasurement.avgTagArea;

        SmartDashboard.putNumber("LL x", pose.getX());
        SmartDashboard.putNumber("LL y", pose.getY());
        SmartDashboard.putNumber("LL posediff", posediff);

        if(pose.getX() == 0)
        {
            return false;
        }
        if(tagcount > 0)
        {
            if(tagcount >= 2)
            {
                xyStdDev = 0.5;
                angleStdDev = 6;
            }
            else if (posediff < 0.8 && tagarea > 0.8)
            {
                xyStdDev = 1;
                angleStdDev = 12;
            }
            else if (posediff < 0.2 && tagarea > 0.1)
            {
                xyStdDev = 2;
                angleStdDev = 30;
            }
        }
        else
        {
            return false;
        }
        Drive.m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, Units.degreesToRadians(angleStdDev)));
        Drive.m_odometry.addVisionMeasurement(pose, timestamp);
        return true;
    }

/*
    public static void EvaluateLimelight(String limelight)
    {
        double timestamp;
        Pose2d pose;
        double latency;
        double tagid;
        double time;

        
        timestamp = Timer.getFPGATimestamp();
        pose = LimelightHelpers.getBotPose2d_wpiBlue(limelight);
        latency = LimelightHelpers.getLatency_Capture(limelight) + LimelightHelpers.getLatency_Pipeline(limelight);
        tagid = LimelightHelpers.getFiducialID(limelight);

        time = (1000 * timestamp - latency) / 1000;

        if(tagid != -1)
        {
            if((Math.abs(pose.getX() - Drive.m_odometry.getEstimatedPosition().getX()) < LimelightConstants.tolerance)
            &&(Math.abs(pose.getY() - Drive.m_odometry.getEstimatedPosition().getY()) < LimelightConstants.tolerance))
            {
                Drive.m_odometry.addVisionMeasurement(pose, time);
            }
        }
    }*/
}
