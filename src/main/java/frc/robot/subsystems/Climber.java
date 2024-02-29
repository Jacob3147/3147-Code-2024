package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climber extends SubsystemBase 
{   
    DoubleSolenoid climbCyl = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
                                                  ClimbConstants.climb_solenoid_port_a, 
                                                  ClimbConstants.climb_solenoid_port_b);


    public Climber()
    {
        
    }

    public void HooksUp()
    {
        climbCyl.set(Value.kForward);
    }

    public void HooksDown()
    {
        climbCyl.set(Value.kReverse);
    }
}
