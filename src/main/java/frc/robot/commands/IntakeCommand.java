package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command 
{
    Intake m_intake;

    public IntakeCommand(Intake intake)
    {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void execute() 
    {
        m_intake.intake_note();
    }

    @Override
    public boolean isFinished() {
        return m_intake.haveNote(); 
    }

    @Override
    public void end(boolean interrupted) 
    {
        m_intake.stop();
    }
}
