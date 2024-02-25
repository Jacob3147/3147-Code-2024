package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drive;

public class Shooter extends SubsystemBase
{
    CANSparkMax topRoll = new CANSparkMax(ShooterConstants.top_motor_port, MotorType.kBrushless);
    CANSparkMax bottomRoll = new CANSparkMax(ShooterConstants.bottom_motor_port, MotorType.kBrushless);
    CANSparkMax tiltMotor = new CANSparkMax(ShooterConstants.tilt_moter_port, MotorType.kBrushless);

    Solenoid stage1 = new Solenoid(PneumaticsModuleType.REVPH, ShooterConstants.stage_1_port);
    Solenoid stage2 = new Solenoid(PneumaticsModuleType.REVPH, ShooterConstants.stage_2_port);

    DutyCycleEncoder tiltEncoder = new DutyCycleEncoder(ShooterConstants.encoder_port);

    ArmFeedforward tiltFF = new ArmFeedforward(ShooterConstants.Ks, ShooterConstants.Kg, ShooterConstants.Kv);
    PIDController tiltPID = new PIDController(ShooterConstants.Kp, ShooterConstants.Ki, ShooterConstants.Kd);

    double angleMeas;

    Pose2d botpose;

    public Shooter()
    {
        bottomRoll.follow(topRoll);

    }

    @Override
    public void periodic() 
    {
        double angle = ShooterConstants.tilt_offset + 360*tiltEncoder.getAbsolutePosition();
        angleMeas = Units.degreesToRadians(angle);
        SmartDashboard.putNumber("shooter angle", angle);

    }
    
    public void spinUp()
    {
        topRoll.set(0.2);
    }

    public void spinDown() 
    {
        topRoll.set(0);
    }

    public boolean shooterAtSpeed()
    {
        return true;
    }

    public void TiltControl(double angleSP)
    {
        double FF = tiltFF.calculate(angleSP, 0);
        double PID = tiltPID.calculate(angleMeas, angleSP);

        //tiltMotor.setVoltage(FF+PID);
    }

    public void lift_amp()
    {
        stage1.set(true);
        stage2.set(false);
    }

    public void lift_trap()
    {
        stage1.set(true);
        stage2.set(true);
    }

    public void lift_speaker()
    {
        stage1.set(false);
        stage2.set(false);
    }


}
