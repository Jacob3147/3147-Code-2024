package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;

    DigitalInput left_limit = new DigitalInput(left_limit_port);
    DigitalInput right_limit = new DigitalInput(right_limit_port);

    SparkPIDController left_PID = climber_left.getPIDController();
    SparkPIDController right_PID = climber_right.getPIDController();

    boolean leftLimit;
    boolean rightLimit;
    
    double leftSpeed = 0;
    double rightSpeed = 0;

    double p = 0.05;

    public Climber(Supplier<Float> rollSupplier)
    {
        this.rollSupplier = rollSupplier;

        climber_left.restoreFactoryDefaults();
        climber_right.restoreFactoryDefaults();

        climber_left.setIdleMode(IdleMode.kBrake);
        climber_right.setIdleMode(IdleMode.kBrake);

        climber_right.setInverted(true);
        
        leftEncoder = climber_left.getEncoder();
        rightEncoder = climber_right.getEncoder();

        climber_left.setSoftLimit(SoftLimitDirection.kReverse, 0);
        climber_right.setSoftLimit(SoftLimitDirection.kReverse, 0);

        rightEncoder.setPositionConversionFactor(climber_rotations_per_mm);
        leftEncoder.setPositionConversionFactor(climber_rotations_per_mm);

        left_PID.setP(p);
        right_PID.setP(p);
    }

    public void periodic() 
    {
        roll = rollSupplier.get();
        SmartDashboard.putNumber("Roll", roll);
        leftLimit = left_limit.get();
        rightLimit = right_limit.get();

        if(leftLimit)
        {
            leftEncoder.setPosition(0);
        }
        if(rightLimit)
        {
            rightEncoder.setPosition(0);
        }

        SmartDashboard.putBoolean("Climber Left Limit", leftLimit);
        SmartDashboard.putBoolean("Climber Right Limit", rightLimit);
        SmartDashboard.putNumber("Climb encoder", leftEncoder.getPosition());
    }

    public void Up()
    {
        left_PID.setReference(-150, ControlType.kPosition);
        //right_PID.setReference(-190, ControlType.kPosition);
    }

    public void Down()
    {
        left_PID.setReference(-50, ControlType.kPosition);
        //right_PID.setReference(-5, ControlType.kPosition);
    }

    public Command SeekZero()
    {
        return Commands.run(
            () -> left_PID.setReference(0.05, ControlType.kDutyCycle))
            .until(() -> leftLimit)
            .andThen(() -> left_PID.setReference(0, ControlType.kDutyCycle));

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
