package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



public class Limelight extends SubsystemBase
{
    NetworkTable m_NetworkTable;

    NetworkTableEntry botpose;
    NetworkTableEntry tid;

    double[] botpose_array = new double[7];
    double tag;
    double limelight_latency;

    Translation2d translation_limelight;
    Rotation2d rotation_limelight;
    Pose2d botpose_limelight;

    HttpCamera limelight_stream;
    
    public Limelight()
    {
        //limelight_stream = new HttpCamera("limelight", "http://limelight.local:5801/stream.mjpg", HttpCameraKind.kMJPGStreamer);
        //CameraServer.startAutomaticCapture(limelight_stream);
        
        m_NetworkTable = NetworkTableInstance.getDefault().getTable("limelight");

        botpose = m_NetworkTable.getEntry("botpose");
        tid = m_NetworkTable.getEntry("tid");
    }

    @Override
    public void periodic() 
    {
        //Apriltag values
        //[x y z pitch roll yaw]
        botpose_array = botpose.getDoubleArray(new double[7]);
        tag = tid.getDouble(-1);

        
        translation_limelight = new Translation2d(botpose_array[0], botpose_array[1]);
        rotation_limelight = new Rotation2d(botpose_array[5]);
        botpose_limelight = new Pose2d(translation_limelight, rotation_limelight);
    
        limelight_latency = Timer.getFPGATimestamp() - (botpose_array[6]/1000.0);

        //Drive.m_odometry.addVisionMeasurement(botpose_limelight, limelight_latency);


        SmartDashboard.putNumber("limelight x", translation_limelight.getX());
        SmartDashboard.putNumber("limelight y", translation_limelight.getY());
        SmartDashboard.putNumber("limelight rotation", rotation_limelight.getDegrees());
    }
}
