package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase
{
    CANSparkMax intake = new CANSparkMax(IntakeConstants.intake_motor_port, MotorType.kBrushless);

    DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 
                                                       IntakeConstants.intake_solenoid_port_a,
                                                       IntakeConstants.intake_solenoid_port_b);

    ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    Color detectedColor;
    double IR;
    
    public Intake()
    {
        
    }
    
    @Override
    public void periodic() 
    {
        detectedColor = colorSensor.getColor();
        IR = colorSensor.getProximity();

        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("IR prox", IR);
    }

    public void intake_note()
    {
        intake.set(1);
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
    public boolean haveNote()
    {
        return false;
    }

}
