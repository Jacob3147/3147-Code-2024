package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SpinUpCommand extends Command
{
    Shooter m_shooter;
    public SpinUpCommand(Shooter shooter)   
    {
        m_shooter = shooter;
    } 

    @Override
    public void initialize() 
    {
        end(false);
        //m_shooter.spinUp();
    }
    
    @Override
    public boolean isFinished() 
    {
        return true;
    }

}
