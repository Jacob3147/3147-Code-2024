// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.commands.DriveTeleopCommand;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;





public class RobotContainer {

    /****** Subsystems ******/
    private final Drive m_DriveSubsystem = new Drive();
    private final Limelight m_LimelightSubsystem = new Limelight();
    private final Arm m_ArmSubsystem = new Arm();
    private final Climber m_ClimberSubsystem = new Climber();
    private final Intake m_IntakeSubsystem = new Intake();
    private final Shooter m_ShooterSubsystem = new Shooter();


    /****** Joysticks and Joystick Suppliers ******/
    private final XboxController m_driverController = new XboxController(0);
    private final XboxController m_operatorController = new XboxController(1);
    Supplier<Double> xSpeedSupplier = () -> m_driverController.getLeftY();
    Supplier<Double> turnSpeedSupplier = () -> m_driverController.getRightX();


    /****** Commands ******/
    private final DriveTeleopCommand m_DriveCommand = new DriveTeleopCommand(m_DriveSubsystem, xSpeedSupplier, turnSpeedSupplier);


    private final SendableChooser<Command> autoChooser;


    
    public RobotContainer() 
    {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);
        
        m_DriveSubsystem.setDefaultCommand(m_DriveCommand);


    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() 
    {
        
        return autoChooser.getSelected();
    }
}
