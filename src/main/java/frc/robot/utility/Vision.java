package frc.robot.utility;

import frc.robot.subsystems.Drive;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Vision
{
    public static boolean EvaluateLimelightNew(String limelight, Field2d field)
    {
        double timestamp;
        Pose2d pose;
        double tagcount;
        double posediff;
        double xyStdDev = 1;
        double angleStdDev = 12;

        

        
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
        pose = limelightMeasurement.pose;
        posediff = Drive.m_odometry.getEstimatedPosition().getTranslation().getDistance(pose.getTranslation());
        timestamp = limelightMeasurement.timestampSeconds;
        tagcount = limelightMeasurement.tagCount;
        SmartDashboard.putNumber(limelight + " diff", posediff);
        if(pose.getX() == 0)
        {
            return false;
        }
        if (posediff < 0.2)
        {
            xyStdDev = 0.5;
            angleStdDev = 15;
        }
        else if (posediff < 0.5)
        {
            xyStdDev = 1;
            angleStdDev = 30;
        }
        else 
        {
            return false;
        }
        SmartDashboard.putNumber(limelight+" posediff", posediff);
       
        Drive.m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, Units.degreesToRadians(angleStdDev)));
        Drive.m_odometry.addVisionMeasurement(pose, timestamp);
        return true;
    }

}