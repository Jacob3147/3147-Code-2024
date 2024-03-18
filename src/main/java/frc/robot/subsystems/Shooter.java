package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;

import java.util.function.Supplier;


public class Shooter extends SubsystemBase
{
    Supplier<Double> angleOffsetTemp;
    public enum ShooterState
    {
        SPEAKER,
        AMP,
        TRAP,
        NEUTRAL,
        AWAIT_HANDOFF,
        PASS,
        SUBWOOFER
    }

    public ShooterState state = ShooterState.NEUTRAL;

    CANSparkMax topRoll = new CANSparkMax(top_motor_port, MotorType.kBrushless);
    CANSparkMax bottomRoll = new CANSparkMax(bottom_motor_port, MotorType.kBrushless);
    CANSparkMax tiltMotor = new CANSparkMax(tilt_moter_port, MotorType.kBrushless);

    DoubleSolenoid stage1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
                                               stage_1_port_a,
                                               stage_1_port_b);

    DoubleSolenoid stage2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
                                               stage_2_port_a,
                                               stage_2_port_b);

    DutyCycleEncoder tiltEncoder = new DutyCycleEncoder(encoder_port);
    ArmFeedforward tiltFF = new ArmFeedforward(Ks, Kg, Kv, Ka);
    ProfiledPIDController tiltPID = new ProfiledPIDController(Kp, Ki, Kd, new TrapezoidProfile.Constraints(maxV, maxA));

    double angleMeas, anglePVradians;
    double target_speaker_angle;

    boolean subwooferOnly = false;

    double lastSpeed = 0;
    double lastTime = Timer.getFPGATimestamp();
    
    public Shooter(Supplier<Double> angleOffset)
    {
        angleOffsetTemp = angleOffset;
        bottomRoll.restoreFactoryDefaults();
        topRoll.restoreFactoryDefaults();
        tiltMotor.restoreFactoryDefaults();

        bottomRoll.setIdleMode(IdleMode.kBrake);
        topRoll.setIdleMode(IdleMode.kBrake);
        tiltMotor.setIdleMode(IdleMode.kCoast);
        bottomRoll.follow(topRoll);

        tiltMotor.setInverted(false);

        SmartDashboard.putBoolean("Subwoofer Only?", false);
    }

    @Override
    public void periodic() 
    {
        subwooferOnly = SmartDashboard.getBoolean("Subwoofer Only?", false);

        target_speaker_angle = calcTiltAngle_Speaker();

        SmartDashboard.putNumber("Speaker angle", target_speaker_angle);
        SmartDashboard.putNumber("dist from speaker", Drive.DistanceToSpeaker());
        
        switch (state) {
            case AWAIT_HANDOFF:
                break;
            case NEUTRAL:
                lift_speaker();
                TiltToAngle(tilt_angle_rest);
                spinDown();
                break;
            case SPEAKER:
                lift_speaker();
                spinUp(shot_power);
                TiltToAngle(target_speaker_angle);
                break;
            case AMP:
                TiltToAngle(tilt_angle_amp);
                lift_amp();
                break;
            case TRAP:
                TiltToAngle(tilt_angle_trap);
                lift_trap();
                break;
            case PASS:
                TiltToAngle(5.0);
                spinUp(1);
                lift_speaker();
            case SUBWOOFER:
                lift_speaker();
                spinUp(shot_power);
                TiltToAngle(angle_1);
                break;
            default:
                break;
        }

        angleMeas = tilt_offset + 360*tiltEncoder.getAbsolutePosition();
        if(angleMeas > 180) {angleMeas -=360;}

        SmartDashboard.putNumber("tilt angle", angleMeas);  

        anglePVradians = -1 * Units.degreesToRadians(angleMeas - 90);
    }
    
    public void TiltToAngle(double angleSP)
    {      
        angleSP -= 20*angleOffsetTemp.get();
        double angleSPradians = -1* Units.degreesToRadians(angleSP - 90);

        double PID = tiltPID.calculate(anglePVradians, angleSPradians);
        
        double acceleration = (tiltPID.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);

        double FF = tiltFF.calculate(tiltPID.getSetpoint().position, tiltPID.getSetpoint().velocity, acceleration);

        tiltMotor.setVoltage(-1*(PID + FF));

        lastSpeed = tiltPID.getSetpoint().velocity;
        lastTime = Timer.getFPGATimestamp();
        SmartDashboard.putNumber("angle PV radians", anglePVradians);
        SmartDashboard.putNumber("angle SP radians", angleSPradians);
    }


    public void spinUp(double speed)
    {
        topRoll.set(-1*RobotController.getBatteryVoltage()/12*speed);
    }

    public void spinDown() 
    {
        topRoll.set(0);
    }    

    public void lift_speaker()
    {
        stage1.set(Value.kReverse);
        stage2.set(Value.kReverse);
    }

    public void lift_amp()
    {
        stage2.set(Value.kForward);
        stage1.set(Value.kReverse); //flipped 3/16 9am
    }

    public void lift_trap()
    {
        stage1.set(Value.kForward);
        stage2.set(Value.kForward);
    }


    private double calcTiltAngle_Speaker()
    {
        double distance = Drive.DistanceToSpeaker();

        if((distance_1 < distance) && (distance <= distance_2))
        {
            return linear_interpolation(distance, 
                                        distance_1, distance_2, 
                                        angle_1, angle_2);
        }
        if((distance_2 < distance) && (distance <= distance_3))
        {
            return linear_interpolation(distance, 
                                        distance_2, distance_3, 
                                        angle_2, angle_3);
        }
        if((distance_3 < distance) && (distance <= distance_4))
        {
            return linear_interpolation(distance, 
                                        distance_3, distance_4, 
                                        angle_3, angle_4);
        }
        if((distance_4 < distance) && (distance <= distance_5))
        {
            return linear_interpolation(distance, 
                                        distance_4, distance_5, 
                                        angle_4, angle_5);
        }
        if((distance_5 < distance) && (distance <= distance_6))
        {
            return linear_interpolation(distance, 
                                        distance_5, distance_6, 
                                        angle_5, angle_6);
        }
        if((distance_6 < distance) && (distance <= distance_7))
        {
            return linear_interpolation(distance, 
                                        distance_6, distance_7, 
                                        angle_6, angle_7);
        }
        if((distance_7 < distance) && (distance <= distance_8))
        {
            return linear_interpolation(distance, 
                                        distance_7, distance_8, 
                                        angle_7, angle_8);
        }
        if((distance_8 < distance) && (distance <= distance_9))
        {
            return linear_interpolation(distance, 
                                        distance_8, distance_9, 
                                        angle_8, angle_9);
        }
        return angle_1;
    }

    private double linear_interpolation(double input, double X1, double X2, double Y1, double Y2)
    {
        return ((Y2-Y1)/(X2-X1))*(input-X1) + Y1;
    }


}
