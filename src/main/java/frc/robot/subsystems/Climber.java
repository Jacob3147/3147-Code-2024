package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climber extends SubsystemBase 
{   
    DoubleSolenoid climbCyl = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
                                                  ClimbConstants.climb_solenoid_port_a, 
                                                  ClimbConstants.climb_solenoid_port_b);

    boolean hooks_are_up = false;

    public Climber()
    {
        
    }
    //having a true and false version feels dumb, but it's to fit with a quierk of command-based
    public BooleanSupplier areHooksUp = () -> {return hooks_are_up; };
    public BooleanSupplier areHooksDown = () -> {return !hooks_are_up; };

    public void HooksUp()
    {
        climbCyl.set(Value.kForward);
        hooks_are_up = true;
    }

    public void HooksDown()
    {
        climbCyl.set(Value.kReverse);
        hooks_are_up = false;
    }
}
