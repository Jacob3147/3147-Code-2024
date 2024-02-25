package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class BloopCommand extends Command
{
    Shooter shooter;
    double start;
    double elapsed;

    public BloopCommand(Shooter shooter)
    {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() 
    {
        start = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() 
    {
        shooter.spinUp(0.2);
    }

    @Override
    public boolean isFinished() 
    {
        return (elapsed > 1);
    }
}
