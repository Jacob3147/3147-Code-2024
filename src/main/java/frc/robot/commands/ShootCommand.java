package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;


public class ShootCommand extends Command 
{
    Shooter m_ShooterSubsystem;
    
    public ShootCommand(Shooter shooter)
    {
        m_ShooterSubsystem = shooter;
        addRequirements(m_ShooterSubsystem);
    }

    @Override
    public void initialize() 
    {
        end(false);
    }

    @Override
    public void execute() 
    {
        
    }

    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
