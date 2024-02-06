package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase
{
    CANSparkMax intake = new CANSparkMax(IntakeConstants.intake_motor_port, MotorType.kBrushless);

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
}
