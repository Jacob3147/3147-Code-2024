package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;


public class Shooter extends SubsystemBase
{
    public enum ShooterState
    {
        SPEAKER,
        AMP,
        TRAP,
        NEUTRAL,
        PENDING
    }

    public ShooterState state = ShooterState.NEUTRAL;

    CANSparkMax topRoll = new CANSparkMax(ShooterConstants.top_motor_port, MotorType.kBrushless);
    CANSparkMax bottomRoll = new CANSparkMax(ShooterConstants.bottom_motor_port, MotorType.kBrushless);
    CANSparkMax tiltMotor = new CANSparkMax(ShooterConstants.tilt_moter_port, MotorType.kBrushless);

    DoubleSolenoid stage1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
                                               ShooterConstants.stage_1_port_a,
                                               ShooterConstants.stage_1_port_b);

    DoubleSolenoid stage2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
                                               ShooterConstants.stage_2_port_a,
                                               ShooterConstants.stage_2_port_b);

    DutyCycleEncoder tiltEncoder = new DutyCycleEncoder(ShooterConstants.encoder_port);
    double Ks, Kg, Kv, Kp, Ki, Kd = 0;
    ArmFeedforward tiltFF = new ArmFeedforward(0, 0.33, 0);
    PIDController tiltPID = new PIDController(Kp, Ki, Kd);

    double angleMeas;
    double target_speaker_angle;

    double target_shooter_speed;

    double angleTest = 0;

    Supplier<Double> angleSrc;
    public Shooter(Supplier<Double> angleSrc)
    {
        bottomRoll.follow(topRoll);
        this.angleSrc = angleSrc;
    }

    @Override
    public void periodic() 
    {
    
        target_shooter_speed = calcShooterSpeed();
        target_speaker_angle = calcTiltAngle_Speaker();

        switch (state) {
            case PENDING:
                break;
            case NEUTRAL:
                lift_speaker();
                TiltToAngle(ShooterConstants.tilt_angle_rest);
                spinDown();
                break;
            case SPEAKER:
                lift_speaker();
                spinUp(target_shooter_speed);
                TiltToAngle(target_speaker_angle);
                break;

            case AMP:
                TiltToAngle(ShooterConstants.tilt_angle_amp);
                lift_amp();
                break;

            case TRAP:
                TiltToAngle(ShooterConstants.tilt_angle_amp);
                lift_trap();
                break;

            default:
                break;
        }

        angleMeas = ShooterConstants.tilt_offset + 360*tiltEncoder.getAbsolutePosition();
        if(angleMeas > 180) {angleMeas -=360;}
        SmartDashboard.putNumber("tilt angle", angleMeas);
        

        tiltPID.setPID(0.05, Ki, Kd);
        
    }
    
    public void spinUp(double speed)
    {
        topRoll.set(-1*speed);
    }

    public void spinDown() 
    {
        topRoll.set(0);
    }

    public boolean shooterAtSpeed()
    {
        return true;
    }

    public void TiltToAngle(double angleSP)
    {
        
        double FF = tiltFF.calculate(Units.degreesToRadians(90-angleSP), 0);
        double PID = tiltPID.calculate(angleMeas, angleSP);
        tiltMotor.setVoltage(FF+PID);

        SmartDashboard.putNumber("Tilt Motor output", FF+PID);
        SmartDashboard.putNumber("tilt angle SP", angleSP);
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

    public void lift_speaker()
    {
        stage1.set(Value.kReverse);
        stage2.set(Value.kReverse);
    }

    private double calcTiltAngle_Speaker()
    {
        double distance = Drive.DistanceToSpeaker();
        SmartDashboard.putNumber("Distance from speaker", distance);

        //return linear_interpolation(distance, ShooterConstants.distance_1, ShooterConstants.distance_3, 
        //                                     ShooterConstants.angle_1, ShooterConstants.angle_3);
        return ShooterConstants.angle_1;
    }

    public double calcShooterSpeed()
    {
        double distance = Drive.DistanceToSpeaker();

        //return linear_interpolation(distance, ShooterConstants.distance_1, ShooterConstants.distance_3, 
        //                                      ShooterConstants.speed_1, ShooterConstants.speed_3);
        return ShooterConstants.speed_1;
    }

    private double linear_interpolation(double input, double X1, double X2, double Y1, double Y2)
    {
        return ((Y2-Y1)/(X2-X1))*(input-X1) + Y1;
    }

    /*private double calcShooterSpeed()
    {
        double distance = Drive.DistanceToSpeaker();

        if((ShooterConstants.distance_1 < distance) && (distance <= ShooterConstants.distance_2))
        {

        }
        if((ShooterConstants.distance_2 < distance) && (distance <= ShooterConstants.distance_3))
        {

        }
        if((ShooterConstants.distance_3 < distance) && (distance <= ShooterConstants.distance_4))
        {

        }
        if((ShooterConstants.distance_4 < distance) && (distance <= ShooterConstants.distance_5))
        {

        }
        if((ShooterConstants.distance_5 < distance) && (distance <= ShooterConstants.distance_6))
        {

        }
        if((ShooterConstants.distance_7 < distance) && (distance <= ShooterConstants.distance_7))
        {

        }
        if((ShooterConstants.distance_1 < distance) && (distance <= ShooterConstants.distance_2))
        {

        }
        
        return 0;
    }

    private double calcTiltAngle_Speaker()
    {
        return 0;
    }
*/

}
