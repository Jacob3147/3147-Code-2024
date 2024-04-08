package frc.robot.utility;

import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Drive;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import java.util.function.Supplier;




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

    Supplier<Double> yawRateSupplier;

    public Vision(Drive drive, Supplier<Double> yawRate)
    {
        this.drive = drive;
        yawRateSupplier = yawRate;
    }

    @Override
    public void periodic() 
    {
        
        if(DriverStation.isTeleop())
        {
            LL_1_hastarget = Evaluate_Limelight(LL1);
            LL_2_hastarget = Evaluate_Limelight(LL2);
        }

        SmartDashboard.putBoolean("LL front valid", LL_1_hastarget);
        SmartDashboard.putBoolean("LL rear valid", LL_2_hastarget);

        LimelightHelpers.SetRobotOrientation("limelight", Drive.m_odometry.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    }

    public boolean Evaluate_Limelight(String LL)
    {
        boolean doRejectUpdate = false;
        double confidence = 0.7;
        

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LL);

        double poseDiff = mt2.pose.getTranslation().getDistance(Drive.m_odometry.getEstimatedPosition().getTranslation());
        drive.field.getObject("LL").setPose(mt2.pose);
        SmartDashboard.putNumber("Posediff " + LL, poseDiff);
        SmartDashboard.putNumber("tagcount " + LL, mt2.tagCount);
        SmartDashboard.putNumber("yaw rate", Math.abs(yawRateSupplier.get()));
        if(Math.abs(yawRateSupplier.get()) > 180) // if our angular velocity is greater than 360 degrees per second, ignore vision updates
        {
            doRejectUpdate = true;
        }
        if(poseDiff > 0.5)
        {
            confidence -= 0.2;
        }
        if(poseDiff > 1)
        {
            doRejectUpdate = true;
        }
        if(mt2.tagCount > 2)
        {
            confidence /= 2;
        }
        if(mt2.tagCount == 0)
        {
            doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
            Drive.m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(confidence,confidence,9999999));
            Drive.m_odometry.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);
        }

        return !doRejectUpdate;

    }

    /*public boolean Evaluate_Limelight(String LL)
    {
        PoseEstimate LL_PoseEstimate = getBotPoseEstimate_wpiBlue(LL);

        int numTags = LL_PoseEstimate.tagCount;
        double confidence = 0;
        double AvgDist = LL_PoseEstimate.avgTagDist;
        double AvgAmbiguity = 0;
        Pose2d LL_pose = LL_PoseEstimate.pose;
        double timestamp = LL_PoseEstimate.timestampSeconds;

        double poseDiff = LL_pose.getTranslation().getDistance(Drive.m_odometry.getEstimatedPosition().getTranslation());
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
    }*/
    
}
