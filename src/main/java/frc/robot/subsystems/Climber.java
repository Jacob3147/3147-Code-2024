package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimbConstants.*;

public class Climber extends SubsystemBase 
{   
    CANSparkMax climber_left = new CANSparkMax(left_climb_port, MotorType.kBrushless);
    CANSparkMax climber_right = new CANSparkMax(left_climb_port, MotorType.kBrushless);

    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;

    SparkPIDController leftPID;
    SparkPIDController rightPID;

    DigitalInput left_limit = new DigitalInput(left_limit_port);
    DigitalInput right_limit = new DigitalInput(right_limit_port);

    boolean leftLimit;
    boolean rightLimit;

    double leftSpeed = 0;
    double rightSpeed = 0;

    double p = 0, i = 0, d = 0, kP = 0, kI = 0, kD = 0;

    public Climber()
    {
        climber_left.restoreFactoryDefaults();
        climber_right.restoreFactoryDefaults();

        climber_left.setIdleMode(IdleMode.kBrake);
        climber_right.setIdleMode(IdleMode.kBrake);

        climber_right.setInverted(true);
        
        leftEncoder = climber_left.getEncoder();
        rightEncoder = climber_right.getEncoder();

        climber_left.setSoftLimit(SoftLimitDirection.kReverse, 0);
        climber_right.setSoftLimit(SoftLimitDirection.kReverse, 0);

        leftEncoder.setPositionConversionFactor(climber_rotations_per_mm);
        leftEncoder.setPositionConversionFactor(climber_rotations_per_mm);

        leftPID = climber_left.getPIDController();
        rightPID = climber_right.getPIDController();

        leftPID.setP(kP);
        leftPID.setI(kI);
        leftPID.setD(kD);

        
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
    }

    public void periodic() 
    {
        p = SmartDashboard.getNumber("P Gain", 0);
        i = SmartDashboard.getNumber("I Gain", 0);
        d = SmartDashboard.getNumber("D Gain", 0);
        if((p != kP)) { leftPID.setP(p); rightPID.setP(p); kP = p; }
        if((i != kI)) { leftPID.setI(i); rightPID.setI(i); kI = i; }
        if((d != kD)) { leftPID.setD(d); rightPID.setD(d); kD = d; }


        leftLimit = left_limit.get();
        rightLimit = right_limit.get();

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
        SmartDashboard.putNumber("Climb PV", leftEncoder.getPosition());
    }

    public void hooksUp()
    {
        
        leftPID.setReference(5, ControlType.kPosition);
        rightPID.setReference(5, ControlType.kPosition);
    }

    public void hooksDown()
    {
        
        leftPID.setReference(1, ControlType.kPosition);
        rightPID.setReference(1, ControlType.kPosition);
    }

    /*
    public void seekZero()
    {
        seekZeroLeft();
        seekZeroRight();
    }

    private void seekZeroLeft()
    {
        if(!leftLimit)
        {
            climber_left.setVoltage(climb_creep_down_spd*12);
        }
        else
        {
            climber_left.setVoltage(0);
            
            return;
        }
    }

    private void seekZeroRight()
    {
        if(!rightLimit)
        {
            climber_right.setVoltage(climb_creep_down_spd*12);
        }
        else
        {
            climber_right.setVoltage(0);
            return;
        }
    }*/


}
