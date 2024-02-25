// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.utility.Vision;
import monologue.Logged;
import monologue.Annotations.Log;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.commands.BloopCommand;
import frc.robot.commands.DriveTeleopCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LiftAmpCommand;
import frc.robot.commands.LiftSpeakerCommand;
import frc.robot.commands.LiftTrapCommand;
import frc.robot.commands.FeedCommand;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;





public class RobotContainer implements Logged {

    
    /****** Subsystems ******/
    private final Drive m_DriveSubsystem = new Drive();
    private final Climber m_ClimberSubsystem = new Climber();
    private final Intake m_IntakeSubsystem = new Intake();
    private final Shooter m_ShooterSubsystem = new Shooter();
    private final LED m_LEDSubsystem = new LED();


    /****** Joysticks and Joystick Suppliers ******/
    private final CommandXboxController m_driverController = new CommandXboxController(Constants.kDriverControllerPort);
    private final XboxController m_operatorController = new XboxController(Constants.kOperatorControllerPort);
    @Log Supplier<Double> xSpeedSupplier = () -> m_driverController.getLeftY();
    @Log Supplier<Double> turnSpeedSupplier = () -> m_driverController.getLeftX();


    /****** Commands ******/
    private final DriveTeleopCommand m_DriveCommand = new DriveTeleopCommand(m_DriveSubsystem, xSpeedSupplier, turnSpeedSupplier);
    private final SpeakerAim_byPose m_SpeakerAim = new SpeakerAim_byPose(m_DriveSubsystem);
    private final IntakeCommand m_IntakeCommand = new IntakeCommand(m_IntakeSubsystem);
    private final SpinUpCommand m_SpinUp = new SpinUpCommand(m_ShooterSubsystem);
    private final SpinDownCommand m_SpinDown = new SpinDownCommand(m_ShooterSubsystem);
    private final FeedCommand m_FeedCommand = new FeedCommand(m_IntakeSubsystem);
    private final LiftAmpCommand m_LiftAmpCommand = new LiftAmpCommand(m_ShooterSubsystem);
    private final LiftSpeakerCommand m_LiftSpeakerCommand = new LiftSpeakerCommand(m_ShooterSubsystem);
    private final LiftTrapCommand m_LiftTrapCommand = new LiftTrapCommand(m_ShooterSubsystem);
    private final BloopCommand m_BloopCommand = new BloopCommand(m_ShooterSubsystem);
;
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() 
    {
        m_DriveSubsystem.setDefaultCommand(m_DriveCommand);

        

        NamedCommands.registerCommand("aim", m_SpeakerAim);
        NamedCommands.registerCommand("intake", m_IntakeCommand);
        NamedCommands.registerCommand("spin up", m_SpinUp);
        NamedCommands.registerCommand("shoot", m_FeedCommand);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("autochooser", autoChooser);


        SmartDashboard.putData("Intake", m_IntakeCommand);
        SmartDashboard.putData("Spin Up", m_SpinUp);
        SmartDashboard.putData("Spin Down", m_SpinDown);
        SmartDashboard.putData("Feed shooter", m_FeedCommand);
        SmartDashboard.putData("Lift amp", m_LiftAmpCommand);
        SmartDashboard.putData("Lift Trap", m_LiftTrapCommand);
        SmartDashboard.putData("Lift Speaker", m_LiftSpeakerCommand);
        SmartDashboard.putData("Bloop", m_BloopCommand);


        configureBindings();
    }

    public void configureBindings()
    {
        m_driverController.leftTrigger(0.5).onTrue(m_IntakeCommand);

        m_driverController.a().onTrue(m_SpeakerAim);

        m_driverController.x().onTrue(Commands.runOnce(() -> m_ShooterSubsystem.state = ShooterState.SPEAKER));

        m_driverController.y().onTrue(Commands.sequence(
            Commands.run(() -> m_IntakeSubsystem.runFwd()),
            Commands.waitSeconds(0.5), 
            Commands.runOnce(() -> m_IntakeSubsystem.stop()),
            Commands.runOnce(() -> m_ShooterSubsystem.state = ShooterState.AMP)
        ));

        m_driverController.b().onTrue(Commands.sequence(
            Commands.runOnce(() -> m_IntakeSubsystem.runFwd()),
            Commands.waitSeconds(0.5), 
            Commands.runOnce(() -> m_IntakeSubsystem.stop()),
            Commands.runOnce(() -> m_ShooterSubsystem.state = ShooterState.TRAP)
        ));
        
        m_driverController.rightTrigger(0.5).and(
            () -> m_ShooterSubsystem.state == ShooterState.SPEAKER)
                .onTrue(Commands.sequence(
                    Commands.runOnce(() -> m_IntakeSubsystem.runFwd()),
                    Commands.waitSeconds(0.5), 
                    Commands.runOnce(() -> m_IntakeSubsystem.stop())
                ));

        m_driverController.rightTrigger(0.5).and(
            () -> m_ShooterSubsystem.state == ShooterState.AMP 
            || m_ShooterSubsystem.state == ShooterState.TRAP)
                .onTrue(Commands.sequence(
                    Commands.runOnce(() -> m_ShooterSubsystem.spinUp(0.5)),
                    Commands.waitSeconds(0.5),
                    Commands.runOnce(() -> m_ShooterSubsystem.spinDown())
                ));

        

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
