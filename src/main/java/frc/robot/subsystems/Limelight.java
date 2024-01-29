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

    NetworkTableEntry botpose_wpiblue;
    NetworkTableEntry botpose_wpired;
    NetworkTableEntry tid;

    double[] botpose_blue = new double[6];
    double[] botpose_red = new double[6];
    double tag;

    Translation2d translation_limelight;
    Rotation2d rotation_limelight;
    Pose2d botpose_limelight;

    HttpCamera limelight_stream;
    
    public Limelight()
    {
        limelight_stream = new HttpCamera("limelight", "http://limelight.local:5801/stream.mjpg", HttpCameraKind.kMJPGStreamer);
        CameraServer.startAutomaticCapture(limelight_stream);
        
        m_NetworkTable = NetworkTableInstance.getDefault().getTable("limelight");

        botpose_wpiblue = m_NetworkTable.getEntry("botpose_wpiblue");
        botpose_wpired = m_NetworkTable.getEntry("botpose_wpired");
        tid = m_NetworkTable.getEntry("tid");
    }

    @Override
    public void periodic() 
    {
        //Apriltag values
        //[x y z pitch roll yaw]
        botpose_blue = botpose_wpiblue.getDoubleArray(new double[6]);
        botpose_red = botpose_wpired.getDoubleArray(new double[6]);
        tag = tid.getDouble(0.0);

        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
        {
            translation_limelight = new Translation2d(botpose_red[0], botpose_red[1]);
            rotation_limelight = new Rotation2d(botpose_red[5]);
            botpose_limelight = new Pose2d(translation_limelight, rotation_limelight);
        }
        else
        {
            translation_limelight = new Translation2d(botpose_blue[0], botpose_blue[1]);
            rotation_limelight = new Rotation2d(botpose_blue[5]);
            botpose_limelight = new Pose2d(translation_limelight, rotation_limelight);
        }

        //Drive.m_odometry.addVisionMeasurement(botpose_limelight, Timer.getFPGATimestamp());

        SmartDashboard.putNumber("limelight x", translation_limelight.getX());
        SmartDashboard.putNumber("limelight y", translation_limelight.getY());
        SmartDashboard.putNumber("limelight rotation", rotation_limelight.getDegrees());
    }
}
