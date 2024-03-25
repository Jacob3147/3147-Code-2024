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

    double p = 0, i = 0, d = 0, kP = 0, kI = 0, kD = 0;

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
        left_PID.setI(i);
        right_PID.setI(i);
        left_PID.setD(d);
        right_PID.setD(d);

        SmartDashboard.putNumber("Climb p", p);
        SmartDashboard.putNumber("Climb i", i);
        SmartDashboard.putNumber("Climb d", d);
    }

    public void periodic() 
    {
        kP = SmartDashboard.getNumber("Climb p", p);
        kI = SmartDashboard.getNumber("Climb i", i);
        kD = SmartDashboard.getNumber("Climb d", d);

        if(kP != p)
        {
            left_PID.setP(kP);
            right_PID.setP(kP);
        }
        if(kI != i)
        {
            left_PID.setI(kI);
            right_PID.setI(kI);
        }
        if(kD != d)
        {
            left_PID.setD(kD);
            right_PID.setD(kD);
        }

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
        left_PID.setReference(3, ControlType.kPosition);
        right_PID.setReference(3, ControlType.kPosition);
    }

    public void Down()
    {
        left_PID.setReference(1, ControlType.kPosition);
        right_PID.setReference(1, ControlType.kPosition);
    }

    public void SeekZero()
    {
        if(!leftLimit)
        {
            left_PID.setReference(-0.03, ControlType.kDutyCycle);
        }
        else
        {
            left_PID.setReference(0, ControlType.kDutyCycle);
        }
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
