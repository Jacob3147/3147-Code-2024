package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class SpeakerAim_byPose extends Command 
{
    Drive m_drive;
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
        targetAngle = Drive.AngleToSpeaker();

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
