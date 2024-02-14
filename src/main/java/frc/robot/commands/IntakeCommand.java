package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command 
{
    Intake m_intake;

    public IntakeCommand(Intake intake)
    {
        m_intake = intake;
    }

    @Override
    public void execute() 
    {
        
    }

    @Override
    public void end(boolean interrupted) 
    {
        
    }
}
