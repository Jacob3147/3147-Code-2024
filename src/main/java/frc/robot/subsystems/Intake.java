package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase
{
    CANSparkMax intake = new CANSparkMax(intake_motor_port, MotorType.kBrushless);

    DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
                                                       intake_solenoid_port_a,
                                                       intake_solenoid_port_b);

    boolean haveNote = false;
    DigitalInput m_beamBreak = new DigitalInput(intake_sensor_port);
    boolean sensorOverride = false;

    public Intake()
    {
        
    }
    
    @Override
    public void periodic() 
    {
        sensorOverride = SmartDashboard.getBoolean("Ignore Intake Sensor?", false);
        haveNote = !m_beamBreak.get();
        SmartDashboard.putBoolean("Have note?", haveNote);
    }

    public void intake_note()
    {
        intake.set(intake_fwd_speed);
        intakeSolenoid.set(Value.kReverse);
    }

    public void feed()
    {
        intake.set(1);
        intakeSolenoid.set(Value.kForward);
    }

    public void stop()
    {
        intake.set(0);
        intakeSolenoid.set(Value.kForward);
    }

    public void reverse()
    {
        intake.set(intake_reverse_speed);
        intakeSolenoid.set(Value.kReverse);
    }

    public Boolean haveNote()
    {
        return sensorOverride ? false : haveNote;
    }

    public Trigger Noted()
    {
        return new Trigger(() -> haveNote());
    }
}
