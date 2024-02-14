package frc.robot;

import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;



public class Vision
{
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
                Drive.m_odometry.addVisionMeasurement(pose, timestamp);
            }
        }
    }
}
