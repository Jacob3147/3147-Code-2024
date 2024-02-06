package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



public class Limelight extends SubsystemBase
{
    NetworkTable m_NetworkTable;

    NetworkTableEntry botpose;
    NetworkTableEntry tid;
    NetworkTableEntry ta;

    double[] botpose_array = new double[7];
    double tag;
    double limelight_latency;
    double tag_area;

    Translation2d translation_limelight;
    Rotation2d rotation_limelight;
    Pose2d botpose_limelight;

    double tag_trust;
    
    public Limelight()
    {
        
        m_NetworkTable = NetworkTableInstance.getDefault().getTable("limelight");

        botpose = m_NetworkTable.getEntry("botpose_wpiblue");
        tid = m_NetworkTable.getEntry("tid");
        ta = m_NetworkTable.getEntry("ta");
    }

    @Override
    public void periodic() 
    {
        //Apriltag values
        //[x y z pitch roll yaw time]
        botpose_array = botpose.getDoubleArray(new double[7]);
        tag = tid.getDouble(-1);
        tag_area = ta.getDouble(0);
        
        translation_limelight = new Translation2d(botpose_array[0], botpose_array[1]);
        rotation_limelight = new Rotation2d(botpose_array[5]);
        botpose_limelight = new Pose2d(translation_limelight, rotation_limelight);
    
        limelight_latency = Timer.getFPGATimestamp() - (botpose_array[6]/1000.0);

        if(tag != -1)
        {
            //trial code to trust the limelight reading the bigger the tag is
            //currently this will give a best stddev of 0.2 meters at 10% tag area, and a worst stddev of 2 meters at 1% tag area.
            tag_trust = -20*tag_area + 2;
            tag_trust = (tag_trust > 2) ? 2 : tag_trust;
            tag_trust = (tag_trust < 0.2) ? 0.2 : tag_trust;
            Drive.m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(tag_trust, tag_trust, Units.degreesToRadians(20*tag_trust))); 
            Drive.m_odometry.addVisionMeasurement(botpose_limelight, limelight_latency);
        }

        SmartDashboard.putNumber("limelight tag area", tag_area);
        SmartDashboard.putNumber("limelight tag", tag);
        SmartDashboard.putNumber("limelight x", translation_limelight.getX());
        SmartDashboard.putNumber("limelight y", translation_limelight.getY());
        SmartDashboard.putNumber("limelight rotation", rotation_limelight.getDegrees());
    }
}
