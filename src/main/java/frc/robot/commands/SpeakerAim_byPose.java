package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class SpeakerAim_byPose extends Command 
{
    Drive m_drive;

    final double blue_speaker_x = 0;
    final double red_speaker_x = Units.feetToMeters(54);
    final double blue_speaker_y = 5.5;
    final double red_speaker_y = 5.5;

    Pose2d currentPose;

    double currentX;
    double currentY;
    double targetAngle;
    boolean result;

    

    public SpeakerAim_byPose(Drive drive)
    {
        this.m_drive = drive;
        addRequirements(drive);
    }

    @Override
    public void execute() 
    {
        currentPose = m_drive.poseSupplier();
        currentX = currentPose.getX();
        currentY = currentPose.getY();

        double xToSpeaker;
        double yToSpeaker;
        if(DriverStation.getAlliance().get() == Alliance.Blue)
        {
            yToSpeaker = blue_speaker_y - currentY;
            xToSpeaker = blue_speaker_x - currentX;
            targetAngle = Units.radiansToDegrees(Math.atan2(yToSpeaker, xToSpeaker));
            
            
        }
        else
        {
            targetAngle = 180 + Units.radiansToDegrees(Math.atan2(blue_speaker_y - currentY, blue_speaker_x - currentX));
        }

        result = m_drive.turnToAngle(targetAngle);
        
        if(result) end(false);
    }

    @Override
    public boolean isFinished() {
        return result;
    }

    @Override
    public void end(boolean interrupted) 
    {
        
    }
}
