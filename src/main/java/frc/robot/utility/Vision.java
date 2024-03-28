package frc.robot.utility;

import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Drive;
import frc.robot.utility.LimelightHelpers.RawFiducial;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.utility.LimelightHelpers.PoseEstimate;
import static frc.robot.utility.LimelightHelpers.getBotPoseEstimate_wpiBlue;
import static frc.robot.utility.LimelightHelpers.RawFiducial;



public class Vision extends SubsystemBase
{
    Drive drive;
    String LL1 = LimelightConstants.limelight_1_name;
    String LL2 = LimelightConstants.limelight_2_name;

    boolean trust;
    double confidence;
    ChassisSpeeds speeds;
    double linearspeed;
    double angularspeed;

    boolean LL_1_hastarget;
    boolean LL_2_hastarget;

    public Vision(Drive drive)
    {
        this.drive = drive;
    }

    @Override
    public void periodic() 
    {


        speeds = drive.speedSupplier();
        linearspeed = Math.abs(speeds.vxMetersPerSecond);
        angularspeed = Math.abs(speeds.omegaRadiansPerSecond);

        trust = (linearspeed < 2) && (angularspeed < 0.5) && DriverStation.isTeleop();
        
        if(trust)
        {
            LL_1_hastarget = Evaluate_Limelight(LL1);
            LL_2_hastarget = Evaluate_Limelight(LL2);
        }

        SmartDashboard.putBoolean("LL front valid", LL_1_hastarget);
        SmartDashboard.putBoolean("LL rear valid", LL_2_hastarget);
    }

    public boolean Evaluate_Limelight(String LL)
    {
        PoseEstimate LL_PoseEstimate = getBotPoseEstimate_wpiBlue(LL);

        int numTags = LL_PoseEstimate.tagCount;
        double confidence = 0;
        double AvgDist = LL_PoseEstimate.avgTagDist;
        double AvgAmbiguity = 0;
        Pose2d LL_pose = LL_PoseEstimate.pose;
        double timestamp = LL_PoseEstimate.timestampSeconds;

        double poseDiff = LL_pose.getTranslation().getDistance(drive.m_odometry.getEstimatedPosition().getTranslation());
        SmartDashboard.putNumber("numTags" + LL, numTags);
        for(RawFiducial tag : LL_PoseEstimate.rawFiducials)
        {
            AvgAmbiguity += tag.ambiguity;
        }
        AvgAmbiguity /= numTags;
        
        if(AvgAmbiguity > 0.5)
        {
            return false;
        }
        if(poseDiff > 0.5)
        {
            return false;
        }

        if(numTags >= 2 && AvgDist < Units.feetToMeters(10))
        {
            confidence = 0.2;
        }
        else if(numTags >= 2 && AvgDist < Units.feetToMeters(20) && poseDiff < 0.3)
        {
            confidence = 0.4;
        }
        else if(AvgDist < Units.feetToMeters(10) && poseDiff < 0.5 && AvgAmbiguity < 0.2)
        {
            confidence = 0.6;
        }
        else if(AvgDist < Units.feetToMeters(20) && poseDiff < 0.2 && AvgAmbiguity < 0.2)
        {
            confidence = 0.8;
        }
        else 
        {
            return false;
        }
        

        Drive.m_odometry.addVisionMeasurement(LL_pose, timestamp, VecBuilder.fill(confidence, confidence, 90));
        drive.field.getObject("LL"+LL).setPose(LL_pose);
        return true;
    }
    
}
/*
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

}*/