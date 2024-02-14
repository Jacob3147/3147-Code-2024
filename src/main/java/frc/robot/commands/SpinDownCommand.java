package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SpinDownCommand extends Command
{
    Shooter m_shooter;
    public SpinDownCommand(Shooter shooter)   
    {
        m_shooter = shooter;
    } 

    @Override
    public void initialize() 
    {
        end(false);
        //m_shooter.spinDown();
    }
    
    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
