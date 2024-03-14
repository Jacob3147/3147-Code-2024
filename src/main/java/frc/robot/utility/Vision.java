package frc.robot.utility;

import frc.robot.subsystems.Drive;
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

}
