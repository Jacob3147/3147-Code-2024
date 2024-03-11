// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.commands.DriveTeleopCommand;
import frc.robot.commands.IntakeCommand;
import static frc.robot.Constants.*;

import frc.robot.commands.SpeakerAim_byPose;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;





public class RobotContainer {
    private final CommandXboxController m_driverController = new CommandXboxController(kDriverControllerPort);
    
    /****** Subsystems ******/
    private final Drive m_DriveSubsystem = new Drive();
    private final Climber m_ClimberSubsystem = new Climber();
    private final Intake m_IntakeSubsystem = new Intake();
    private final Shooter m_ShooterSubsystem = new Shooter();
    private final LED m_LEDSubsystem = new LED();


    /****** Joysticks and Joystick Suppliers ******/
   
   // private final XboxController m_operatorController = new XboxController(Constants.kOperatorControllerPort);
    Supplier<Double> xSpeedSupplier = () -> m_driverController.getLeftY();
    Supplier<Double> turnSpeedSupplier = () -> m_driverController.getLeftX();
    


    /****** Commands ******/
    private final DriveTeleopCommand m_DriveCommand = new DriveTeleopCommand(m_DriveSubsystem, xSpeedSupplier, turnSpeedSupplier);
    private final SpeakerAim_byPose m_SpeakerAim = new SpeakerAim_byPose(m_DriveSubsystem);
    private final IntakeCommand m_IntakeCommand = new IntakeCommand(m_IntakeSubsystem);

    private final SendableChooser<Command> autoChooser;

    public Command testSequence()
    {
        return Commands.sequence(
            Commands.runOnce(() -> m_ShooterSubsystem.lift_amp()),
            Commands.waitSeconds(5),
            Commands.runOnce(() -> m_ShooterSubsystem.lift_trap()),
            Commands.waitSeconds(5),
            Commands.runOnce(() -> m_ShooterSubsystem.lift_speaker()),
            Commands.waitSeconds(2),
            Commands.runOnce(() -> m_ShooterSubsystem.spinUp(1)),
            Commands.waitSeconds(5),
            Commands.runOnce(() -> m_ShooterSubsystem.spinDown()),
            Commands.waitSeconds(5),
            Commands.runOnce(() -> m_ShooterSubsystem.TiltToAngle(-20)),
            Commands.waitSeconds(5),
            Commands.runOnce(() -> m_ShooterSubsystem.TiltToAngle(0)),
            Commands.waitSeconds(5),
            Commands.runOnce(() -> m_DriveSubsystem.setDriveMotors(new ChassisSpeeds(0.5, 0, 0))),
            Commands.waitSeconds(5),
            Commands.runOnce(() -> m_DriveSubsystem.setDriveMotors(new ChassisSpeeds(0, 0, 0))),
            Commands.waitSeconds(5),
            m_IntakeCommand
        );


        
    }
    public RobotContainer() 
    {
        m_DriveSubsystem.setDefaultCommand(m_DriveCommand);

        m_LEDSubsystem.LED_Red();

        NamedCommands.registerCommand("aim", m_SpeakerAim);
        NamedCommands.registerCommand("intake", m_IntakeCommand);
        NamedCommands.registerCommand(
            "speaker",
                    Commands.runOnce(() -> m_ShooterSubsystem.state = ShooterState.SPEAKER)
        );

        NamedCommands.registerCommand(
            "shoot", 
                Commands.sequence(
                    Commands.runOnce(() -> m_IntakeSubsystem.feed()),
                    Commands.waitSeconds(0.5), 
                    Commands.runOnce(() -> m_IntakeSubsystem.stop())

                    //Commands.waitSeconds(1)
                    //Commands.runOnce(() -> m_ShooterSubsystem.state = ShooterState.NEUTRAL)                    
                )
        );

        
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("autochooser", autoChooser);


        configureBindings();
    }

    

    public void configureBindings()
    {
        m_driverController.leftTrigger(0.5).whileTrue(m_IntakeCommand);

        m_driverController.a().onTrue(m_SpeakerAim);

        m_driverController.x().onTrue(Commands.runOnce(() -> m_ShooterSubsystem.state = ShooterState.SPEAKER));

        
        m_driverController.y().onTrue(Commands.sequence(
            Commands.runOnce(() -> m_ShooterSubsystem.state = ShooterState.AWAIT_HANDOFF),
            Commands.runOnce(() -> m_ShooterSubsystem.spinUp(amptrap_handoff_shooter_speed)),
            Commands.waitSeconds(0.1),
            Commands.runOnce(() -> m_IntakeSubsystem.feed()),
            Commands.waitSeconds(amptrap_handoff_feed_time), 
            Commands.runOnce(() -> m_ShooterSubsystem.spinDown()),
            Commands.runOnce(() -> m_IntakeSubsystem.stop()),
            Commands.runOnce(() -> m_ShooterSubsystem.state = ShooterState.AMP)
        ));
        /* 
        m_driverController.b().onTrue(
            Commands.runOnce(() -> m_ShooterSubsystem.state = ShooterState.PASS)
        );*/

        m_driverController.b().onTrue(Commands.sequence(
            Commands.runOnce(() -> m_ShooterSubsystem.state = ShooterState.AWAIT_HANDOFF),
            Commands.runOnce(() -> m_ShooterSubsystem.spinUp(amptrap_handoff_shooter_speed)),
            Commands.waitSeconds(0.1),
            Commands.runOnce(() -> m_IntakeSubsystem.feed()),
            Commands.waitSeconds(amptrap_handoff_feed_time), 
            Commands.runOnce(() -> m_ShooterSubsystem.spinDown()),
            Commands.runOnce(() -> m_IntakeSubsystem.stop()),
            Commands.runOnce(() -> m_ShooterSubsystem.state = ShooterState.TRAP_PRE),
            Commands.waitSeconds(1),
            Commands.runOnce(() -> m_ShooterSubsystem.state = ShooterState.TRAP_FULL)
        ));
        
        m_driverController.rightTrigger(0.5).and(
            () -> m_ShooterSubsystem.state == ShooterState.SPEAKER || m_ShooterSubsystem.state == ShooterState.PASS)
                .onTrue(Commands.sequence(
                    Commands.runOnce(() -> m_IntakeSubsystem.feed()),
                    Commands.waitSeconds(speaker_feed_time), 
                    Commands.runOnce(() -> m_IntakeSubsystem.stop()),
                    Commands.waitSeconds(1),
                    Commands.runOnce(() -> m_ShooterSubsystem.state = ShooterState.NEUTRAL)
                ));
        
        m_driverController.rightTrigger(0.5).and(
            () -> m_ShooterSubsystem.state == ShooterState.AMP)
                .onTrue(Commands.sequence(
                    Commands.runOnce(() -> m_ShooterSubsystem.spinUp(amptrap_deposit_speed)),
                    Commands.waitSeconds(amptrap_deposit_time),
                    Commands.runOnce(() -> m_ShooterSubsystem.spinDown()),
                    Commands.runOnce(() -> m_ShooterSubsystem.state = ShooterState.NEUTRAL)
                ));

        m_driverController.rightTrigger(0.5).and(
            () -> m_ShooterSubsystem.state == ShooterState.TRAP_FULL)
                .onTrue(Commands.sequence(
                    Commands.runOnce(() -> m_ShooterSubsystem.spinUp(amptrap_deposit_speed)),
                    Commands.waitSeconds(amptrap_deposit_time),
                    Commands.runOnce(() -> m_ShooterSubsystem.spinDown()),
                    Commands.runOnce(() -> m_ShooterSubsystem.state = ShooterState.TRAP_POST),
                    Commands.waitSeconds(1),
                    Commands.runOnce(() -> m_ShooterSubsystem.state = ShooterState.NEUTRAL)
                ));


        m_driverController.leftBumper()
        .onTrue(
            Commands.runOnce(() -> m_IntakeSubsystem.reverse()));
        m_driverController.leftBumper()
        .onFalse(
            Commands.runOnce(() -> m_IntakeSubsystem.stop()));

        m_IntakeSubsystem.Noted().onTrue(
            Commands.parallel(
                Commands.sequence(
                    Commands.runOnce(() -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, rumble_intensity)),
                    Commands.waitSeconds(rumble_time),
                    Commands.runOnce(() -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0))
                ),
                Commands.sequence(
                    Commands.runOnce(() -> m_LEDSubsystem.flashOrange()),
                    Commands.waitSeconds(1),
                    Commands.runOnce(() -> m_LEDSubsystem.LED_Orange())
                )
        ));

        m_IntakeSubsystem.Noted().onFalse(Commands.runOnce(() -> m_LEDSubsystem.LED_Red()));

    }

    //called on teleopInit, kills lingering auto spinup
    void teleopResets()
    {
        m_ShooterSubsystem.state = ShooterState.NEUTRAL;
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
