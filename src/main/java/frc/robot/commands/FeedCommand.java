package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;



public class FeedCommand extends Command 
{
    
    Intake m_IntakeSubsystem;
    double start;
    double elapsed;
    
    public FeedCommand(Intake intake)
    {
        m_IntakeSubsystem = intake;
        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void initialize() 
    {
        start = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() 
    {
        elapsed = Timer.getFPGATimestamp() - start;
        m_IntakeSubsystem.runFwd();
    }

    @Override
    public boolean isFinished() 
    {
        return (elapsed > 1);
    }
}
