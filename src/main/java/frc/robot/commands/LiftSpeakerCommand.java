package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class LiftSpeakerCommand extends Command
{
    Shooter m_Shooter;

    public LiftSpeakerCommand(Shooter m_Shooter)
    {
        this.m_Shooter = m_Shooter;
        addRequirements(m_Shooter);
    }

    @Override
    public void initialize() 
    {
        m_Shooter.lift_speaker();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}