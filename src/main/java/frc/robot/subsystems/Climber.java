package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climber extends SubsystemBase 
{
    Solenoid leftClimb = new Solenoid(PneumaticsModuleType.REVPH, ClimbConstants.left_climb_port);
    Solenoid rightClimb = new Solenoid(PneumaticsModuleType.REVPH, ClimbConstants.right_climb_port);

    public Climber()
    {
        
    }

    public void HooksUp()
    {
        leftClimb.set(true);
        rightClimb.set(true);
    }

    public void HooksDown()
    {
        leftClimb.set(false);
        rightClimb.set(false);
    }
}
