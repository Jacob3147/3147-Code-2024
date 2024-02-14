// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import monologue.Logged;
import monologue.Annotations.Log;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.commands.DriveTeleopCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SpeakerAim_byPose;
import frc.robot.commands.SpinDownCommand;
import frc.robot.commands.SpinUpCommand;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;





public class RobotContainer implements Logged {

    
    /****** Subsystems ******/
    private final Drive m_DriveSubsystem = new Drive();
    private final Limelight m_LimelightSubsystem = new Limelight();
    private final Climber m_ClimberSubsystem = new Climber();
    private final Intake m_IntakeSubsystem = new Intake();
    private final Shooter m_ShooterSubsystem = new Shooter();


    /****** Joysticks and Joystick Suppliers ******/
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final XboxController m_operatorController = new XboxController(1);
    @Log Supplier<Double> xSpeedSupplier = () -> m_driverController.getLeftY();
    @Log Supplier<Double> turnSpeedSupplier = () -> m_driverController.getRightX();


    /****** Commands ******/
    private final DriveTeleopCommand m_DriveCommand = new DriveTeleopCommand(m_DriveSubsystem, xSpeedSupplier, turnSpeedSupplier);
    private final SpeakerAim_byPose m_SpeakerAim = new SpeakerAim_byPose(m_DriveSubsystem);
    private final IntakeCommand m_IntakeCommand = new IntakeCommand(m_IntakeSubsystem);
    private final SpinUpCommand m_SpinUp = new SpinUpCommand(m_ShooterSubsystem);
    private final SpinDownCommand m_SpinDown = new SpinDownCommand(m_ShooterSubsystem);
    private final ShootCommand m_ShootCommand = new ShootCommand(m_ShooterSubsystem);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() 
    {
       
        
        m_DriveSubsystem.setDefaultCommand(m_DriveCommand);

        

        NamedCommands.registerCommand("aim", m_SpeakerAim);
        NamedCommands.registerCommand("intake", m_IntakeCommand);
        NamedCommands.registerCommand("spin up", m_SpinUp);
        NamedCommands.registerCommand("shoot", m_ShootCommand);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("autochooser", autoChooser);

        configureBindings();
    }

    public void configureBindings()
    {
        m_driverController.a().onTrue(m_SpeakerAim);
        m_driverController.rightTrigger(0.5).onTrue(m_ShootCommand);
        m_driverController.leftTrigger(0.5).whileTrue(m_IntakeCommand);

    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    @Log
    public Command getAutonomousCommand() 
    {
        
        return autoChooser.getSelected();
    }
}
