package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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


public class Shooter extends SubsystemBase
{
    public enum ShooterState
    {
        SPEAKER,
        AMP,
        TRAP_PRE,
        TRAP_FULL,
        TRAP_POST,
        NEUTRAL,
        AWAIT_HANDOFF,
        PASS
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
    double Ks = 0;
    double Kg = 0.33;
    double Kv = 1;
    double Ka = 0;
    double Kp, Ki, Kd, p, i, d = 0;
    ArmFeedforward tiltFF = new ArmFeedforward(Ks, Kg, Kv, Ka);
    ProfiledPIDController tiltPID = new ProfiledPIDController(Kp, Ki, Kd, new TrapezoidProfile.Constraints(maxV, maxA));

    double angleMeas, anglePVradians;
    double target_speaker_angle;
    double target_shooter_speed;
    double angleTest = 0;

    RelativeEncoder shot_speed_encoder = topRoll.getEncoder();

    boolean subwooferOnly = false;

    double lastSpeed = 0;
    double lastTime = Timer.getFPGATimestamp();
    
    public Shooter()
    {
        bottomRoll.restoreFactoryDefaults();
        topRoll.restoreFactoryDefaults();
        tiltMotor.restoreFactoryDefaults();

        bottomRoll.setIdleMode(IdleMode.kBrake);
        topRoll.setIdleMode(IdleMode.kBrake);
        tiltMotor.setIdleMode(IdleMode.kBrake);
        bottomRoll.follow(topRoll);

        SmartDashboard.putBoolean("Subwoofer Only?", false);

        SmartDashboard.putNumber("Kp",Kp);
        SmartDashboard.putNumber("Ki",Ki);
        SmartDashboard.putNumber("Kd",Kd);
        SmartDashboard.putNumber("Ks",Ks);
        SmartDashboard.putNumber("Kg",Kg);
        SmartDashboard.putNumber("Kv",Kv);
        SmartDashboard.putNumber("Ka",Ka);
    }

    @Override
    public void periodic() 
    {
        subwooferOnly = SmartDashboard.getBoolean("Subwoofer Only?", false);

        target_shooter_speed = calcShooterSpeed();
        target_speaker_angle = calcTiltAngle_Speaker();

        SmartDashboard.putNumber("Target Speed", target_shooter_speed);
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
                spinUp(target_shooter_speed);
                TiltToAngle(target_speaker_angle);
                break;
            case AMP:
                TiltToAngle(tilt_angle_amp);
                lift_amp();
                break;
            case TRAP_PRE:
                TiltToAngle(tilt_angle_trap);
                break;
            case TRAP_FULL:
                TiltToAngle(tilt_angle_trap);
                lift_trap();
                break;
            case TRAP_POST:
                lift_speaker();
            case PASS:
                TiltToAngle(5.0);
                spinUp(1);
                lift_speaker();
            default:
                break;
        }

        angleMeas = tilt_offset + 360*tiltEncoder.getAbsolutePosition();
        if(angleMeas > 180) {angleMeas -=360;}

        SmartDashboard.putNumber("tilt angle", angleMeas);  

        anglePVradians = Units.degreesToRadians(angleMeas - 90);

        
        p = SmartDashboard.getNumber("Kp",Kp);
        i = SmartDashboard.getNumber("Ki",Ki);
        d = SmartDashboard.getNumber("Kd",Kd);
        

        if(p != Kp) tiltPID.setP(p);
        if(i != Ki) tiltPID.setI(i);
        if(d != Kd) tiltPID.setD(i);
        
    }
    
    public void TiltToAngle(double angleSP)
    {      
        
        double angleSPradians = Units.degreesToRadians(angleSP - 90);

        double PID = tiltPID.calculate(anglePVradians, angleSPradians);
        
        double acceleration = (tiltPID.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);

        double FF = tiltFF.calculate(tiltPID.getSetpoint().position, tiltPID.getSetpoint().velocity, acceleration);

        tiltMotor.setVoltage(PID + FF);

        lastSpeed = tiltPID.getSetpoint().velocity;
        lastTime = Timer.getFPGATimestamp();
        SmartDashboard.putNumber("tilt angle SP", angleSP);
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

    public boolean shooterAtSpeed()
    {
        return true;
    }

    

    public void lift_speaker()
    {
        stage1.set(Value.kReverse);
        stage2.set(Value.kReverse);
    }

    public void lift_amp()
    {
        stage1.set(Value.kForward);
        stage2.set(Value.kReverse);
    }

    public void lift_trap()
    {
        stage1.set(Value.kForward);
        stage2.set(Value.kForward);
    }


    private double calcShooterSpeed()
    {
        return shot_power;
    }

    private double calcTiltAngle_Speaker()
    {
        double distance = subwooferOnly ? 1.3 : Drive.DistanceToSpeaker();
        double velocity = shot_power*(5800*0.8 / 60) * wheel_diameter*Math.PI;

        double angle = 
        (180 / Math.PI ) 
        * Math.atan(
            (Math.pow(velocity, 2)
            - Math.sqrt(Math.pow(velocity,4)-9.8*(
                9.8*Math.pow(distance,2)+2*Math.pow(velocity,2)*(speaker_height-shooter_height))
                )
            )
            / (9.8*distance)
        );
  
        return -angle;
    }

    /*private double calcShooterSpeed()
    {
        double distance = Drive.DistanceToSpeaker();

        if((distance_1 < distance) && (distance <= distance_2))
        {
            return linear_interpolation(distance, 
                                        distance_1, distance_2, 
                                        speed_1, speed_2);
        }
        if((distance_2 < distance) && (distance <= distance_3))
        {
            return linear_interpolation(distance, 
                                        distance_2, distance_3, 
                                        speed_2, speed_3);
        }
        if((distance_3 < distance) && (distance <= distance_4))
        {
            return linear_interpolation(distance, 
                                        distance_3, distance_4, 
                                        speed_3, speed_4);
        }
        if((distance_4 < distance) && (distance <= distance_5))
        {
            return linear_interpolation(distance, 
                                        distance_4, distance_5, 
                                        speed_4, speed_5);
        }
        if((distance_5 < distance) && (distance <= distance_6))
        {
            return linear_interpolation(distance, 
                                        distance_5, distance_6, 
                                        speed_5, speed_6);
        }
        if((distance_6 < distance) && (distance <= distance_7))
        {
            return linear_interpolation(distance, 
                                        distance_6, distance_7, 
                                        speed_6, speed_7);
        }
        if((distance_7 < distance) && (distance <= distance_8))
        {
            return linear_interpolation(distance, 
                                        distance_7, distance_8, 
                                        speed_7, speed_8);
        }
        if((distance_8 < distance) && (distance <= distance_9))
        {
            return linear_interpolation(distance, 
                                        distance_8, distance_9, 
                                        speed_8, speed_9);
        }
        return speed_1;
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
    }*/


}
