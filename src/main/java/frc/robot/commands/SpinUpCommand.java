package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SpinUpCommand extends Command
{
    Shooter m_shooter;
    public SpinUpCommand(Shooter shooter)   
    {
        m_shooter = shooter;
        addRequirements(m_shooter);
    } 

    @Override
    public void initialize() 
    {
        m_shooter.spinUp();
    }
    
    @Override
    public boolean isFinished() 
    {
        return m_shooter.shooterAtSpeed();
    }

    @Override
    public void end(boolean interrupted) {
        
    }

}
