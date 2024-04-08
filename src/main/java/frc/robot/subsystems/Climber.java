package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimbConstants.*;

import java.util.function.Supplier;

public class Climber extends SubsystemBase 
{   
    CANSparkMax climber_left = new CANSparkMax(left_climb_port, MotorType.kBrushless);
    CANSparkMax climber_right = new CANSparkMax(right_climb_port, MotorType.kBrushless);

    //Negative roll = left side low
    Supplier<Float> rollSupplier;
    double roll;


    public Climber(Supplier<Float> rollSupplier)
    {
        this.rollSupplier = rollSupplier;

        climber_left.restoreFactoryDefaults();
        climber_right.restoreFactoryDefaults();

        climber_left.setIdleMode(IdleMode.kBrake);
        climber_right.setIdleMode(IdleMode.kBrake);

        climber_right.setInverted(true);
        
    }

    public void periodic() 
    {
        roll = rollSupplier.get();
    }

    public void jogUp()
    {
        climber_left.set(-climbSpeed);
        climber_right.set(-climbSpeed);

    }

    public void jogDown()
    {
        if(roll < -2)
        {
            climber_left.set(climbSpeed);
            climber_right.set(0.5*climbSpeed);
        }
        else if(roll > 2)
        {
            climber_left.set(0.5*climbSpeed);
            climber_right.set(climbSpeed);
        }
        else 
        {
            climber_left.set(climbSpeed);
            climber_right.set(climbSpeed);
        }
        
    }

    public void leftJogDown()
    {
        climber_left.set(climbSpeed);
    }

    public void rightJogDown()
    {
            climber_right.set(climbSpeed);
    }

    public void stop()
    {
        climber_left.set(0);
        climber_right.set(0);
    }    
    

  

}
