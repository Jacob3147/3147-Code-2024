package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase
{
    CANSparkMax topRoll = new CANSparkMax(ShooterConstants.top_motor_port, MotorType.kBrushless);
    CANSparkMax bottomRoll = new CANSparkMax(ShooterConstants.bottom_motor_port, MotorType.kBrushless);

    public Shooter()
    {
        bottomRoll.follow(topRoll);

    }
    
    public void spinUp()
    {

    }

    public void spinDown() 
    {
        
    }


}
