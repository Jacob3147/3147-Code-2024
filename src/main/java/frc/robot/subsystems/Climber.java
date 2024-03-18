package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimbConstants.*;

import java.util.function.Supplier;

public class Climber extends SubsystemBase 
{   
    CANSparkMax climber_left = new CANSparkMax(left_climb_port, MotorType.kBrushless);
    CANSparkMax climber_right = new CANSparkMax(right_climb_port, MotorType.kBrushless);

    Supplier<Float> rollSupplier;
    //Negative roll = left side low
    double roll;
    /*RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;

    DigitalInput left_limit = new DigitalInput(left_limit_port);
    DigitalInput right_limit = new DigitalInput(right_limit_port);

    boolean leftLimit;
    boolean rightLimit;
    
    double leftSpeed = 0;
    double rightSpeed = 0;

    double p = 0, i = 0, d = 0, kP = 0, kI = 0, kD = 0;*/

    public Climber(Supplier<Float> rollSupplier)
    {
        this.rollSupplier = rollSupplier;

        climber_left.restoreFactoryDefaults();
        climber_right.restoreFactoryDefaults();

        climber_left.setIdleMode(IdleMode.kBrake);
        climber_right.setIdleMode(IdleMode.kBrake);

        climber_right.setInverted(true);
        
        //leftEncoder = climber_left.getEncoder();
        //rightEncoder = climber_right.getEncoder();


        //rightEncoder.setPositionConversionFactor(climber_rotations_per_mm);
        //leftEncoder.setPositionConversionFactor(climber_rotations_per_mm);

    }

    public void periodic() 
    {
        roll = rollSupplier.get();
        SmartDashboard.putNumber("Roll", roll);
        /*leftLimit = false; //left_limit.get();
        rightLimit = false; //right_limit.get();

        if(leftLimit)
        {
            leftEncoder.setPosition(0);
            climber_left.stopMotor();
        }
        if(rightLimit)
        {
            rightEncoder.setPosition(0);
            climber_right.stopMotor();
        }

        SmartDashboard.putBoolean("Climber Left Limit", leftLimit);
        SmartDashboard.putBoolean("Climber Right Limit", rightLimit);
        SmartDashboard.putNumber("Climb encoder", leftEncoder.getPosition());*/
    }


    public void hooksUp()
    {
        climber_left.set(-1.2*climbSpeed);
        climber_right.set(-climbSpeed);

    }

    public void hooksDown()
    {
        if(roll < -2)
        {
            climber_left.set(1.2*climbSpeed);
            climber_right.set(0.5*climbSpeed);
        }
        else if(roll > 2)
        {
            climber_left.set(1.2*0.5*climbSpeed);
            climber_right.set(climbSpeed);
        }
        else 
        {
            climber_left.set(1.2*climbSpeed);
            climber_right.set(climbSpeed);
        }
        
    }

    public void leftDown()
    {
        climber_left.set(climbSpeed);
    }

    public void rightDown()
    {
        climber_right.set(climbSpeed);
    }

    public void stop()
    {
        climber_left.set(0);
        climber_right.set(0);
    }    
    

  

}
