// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.utility.LED;
import frc.robot.utility.Vision;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.commands.DriveTeleopCommand;
import frc.robot.commands.IntakeCommand;
import static frc.robot.Constants.*;

import frc.robot.commands.SpeakerAim_byPose;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;




public class RobotContainer {

    /***** Driver controller and suppliers for controller axes *****/
    /***** The suppliers are passed to commands/subsystems instead of passing the entire joystick *****/
    /***** CommandXboxController creates the buttons as Triggers instead of booleans for easy command mapping *****/
    private final CommandXboxController m_driverController = new CommandXboxController(kDriverControllerPort);
    Supplier<Double> xSpeedSupplier = () -> m_driverController.getLeftY();
    Supplier<Double> turnSpeedSupplier = () -> m_driverController.getLeftX();
    Supplier<Double> angleOffset = () -> m_driverController.getRightY();

    /***** Gyro and gyro suppliers *****/
    private static final AHRS navX = new AHRS(I2C.Port.kMXP);
    Supplier<Rotation2d> gyroRotation = () -> navX.getRotation2d();
    Supplier<Float> rollSupplier = () -> navX.getPitch();
    Supplier<Double> yawRateSupplier = () -> navX.getRate();

    /***** Subsystems *****/
    private final Drive m_DriveSubsystem = new Drive(gyroRotation);
    private final Climber m_ClimberSubsystem = new Climber(rollSupplier);
    private final Intake m_IntakeSubsystem = new Intake();
    private final Shooter m_ShooterSubsystem = new Shooter(angleOffset);   
    private final Vision m_visionSubsystem = new Vision(m_DriveSubsystem, yawRateSupplier); 
    public final LED m_LED = new LED(m_IntakeSubsystem.haveNoteSupplier());

    /***** Commands *****/
    /***** As a reminder, commands can be written as an entire class that extends Command like these, or as short in-line commands *****/
    /***** Typically we use classes for complicated commands, like if they need the full initialize, execute, isfinished, end functions *****/
    /***** Simpler functions can just do the pattern of Commands.runOnce(() -> subsystem.function()) *****/
    private final DriveTeleopCommand m_DriveCommand = new DriveTeleopCommand(m_DriveSubsystem, xSpeedSupplier, turnSpeedSupplier);
    private final SpeakerAim_byPose m_SpeakerAim = new SpeakerAim_byPose(m_DriveSubsystem);
    private final IntakeCommand m_IntakeCommand = new IntakeCommand(m_IntakeSubsystem);
    
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() 
    {
    
        m_DriveSubsystem.setDefaultCommand(m_DriveCommand);

        //This is how commands interact with pathplanner
        NamedCommands.registerCommand("aim", m_SpeakerAim);
        NamedCommands.registerCommand("intake", m_IntakeCommand);
        NamedCommands.registerCommand("speaker", Commands.runOnce(() -> m_ShooterSubsystem.state = ShooterState.SPEAKER));

        NamedCommands.registerCommand("shoot", 
            Commands.sequence(
                Commands.runOnce(() -> m_IntakeSubsystem.feed()),
                Commands.waitSeconds(0.5), 
                Commands.runOnce(() -> m_IntakeSubsystem.stop())             
            )
        );
        
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("autochooser", autoChooser);


        configureBindings();
    }

    
    /*
        This maps Triggers to Commands 
        Remember, this only runs once at start of code 
        After that, the command scheduler handles listening for Triggers 

        The main pattern is Trigger.onTrue(Command) or Trigger.whileTrue(Command) 
        Triggers can be chained like Trigger1.and(Trigger2).onTrue(Command)

        Both Triggers and Commands are functional interfaces, so they can be substituted with lambdas () ->
        new Trigger (() -> boolean).onTrue(Command)
        Trigger.onTrue(() -> Commands.runOnce(function()))

        Alternately, you could have a method in a subsystem that returns a Trigger or a Command
        For example intake subsystem has a function "public Trigger Noted()" that returns a trigger for the intake sensor
    */
    public void configureBindings()
    {
        
        m_driverController.leftTrigger(0.5).whileTrue(m_IntakeCommand);

        m_driverController.a().and(() -> m_ShooterSubsystem.state == ShooterState.SPEAKER).whileTrue(m_SpeakerAim);

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

        /* Unused trap
        m_driverController.b().onTrue(Commands.sequence(
            Commands.runOnce(() -> m_ShooterSubsystem.state = ShooterState.AWAIT_HANDOFF),
            Commands.runOnce(() -> m_ShooterSubsystem.spinUp(amptrap_handoff_shooter_speed)),
            Commands.waitSeconds(0.1),
            Commands.runOnce(() -> m_IntakeSubsystem.feed()),
            Commands.waitSeconds(amptrap_handoff_feed_time), 
            Commands.runOnce(() -> m_ShooterSubsystem.spinDown()),
            Commands.runOnce(() -> m_IntakeSubsystem.stop()),
            Commands.runOnce(() -> m_ShooterSubsystem.state = ShooterState.TRAP)
        ));*/

        m_driverController.b().onTrue(Commands.runOnce(() -> m_ShooterSubsystem.state = ShooterState.SUBWOOFER));
        
        m_driverController.rightTrigger(0.5).and(
            () -> m_ShooterSubsystem.state == ShooterState.SPEAKER || m_ShooterSubsystem.state == ShooterState.PASS || m_ShooterSubsystem.state == ShooterState.SUBWOOFER)
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
                    Commands.runOnce(() -> m_ShooterSubsystem.spinUp(amp_deposit_speed)),
                    Commands.waitSeconds(amptrap_deposit_time),
                    Commands.runOnce(() -> m_ShooterSubsystem.spinDown()),
                    Commands.runOnce(() -> m_ShooterSubsystem.state = ShooterState.NEUTRAL)
                ));

        m_driverController.rightTrigger(0.5).and(
            () -> m_ShooterSubsystem.state == ShooterState.TRAP)
                .onTrue(Commands.sequence(
                    Commands.runOnce(() -> m_ShooterSubsystem.spinUp(trap_deposit_speed)),
                    Commands.waitSeconds(amptrap_deposit_time),
                    Commands.runOnce(() -> m_ShooterSubsystem.spinDown())

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
                    Commands.runOnce(() -> m_LED.flashing_note = true),
                    Commands.waitSeconds(2),
                    Commands.runOnce(() -> m_LED.flashing_note = false)
                )
            )
        );
        
        
    
        m_driverController.povUp().whileTrue(Commands.runOnce(() -> m_ClimberSubsystem.jogUp()));
        m_driverController.povUp().onFalse(Commands.runOnce(() -> m_ClimberSubsystem.stop()));

        m_driverController.povDown().whileTrue(Commands.runOnce(() -> m_ClimberSubsystem.jogDown()));
        m_driverController.povDown().onFalse(Commands.runOnce(() -> m_ClimberSubsystem.stop()));

        m_driverController.povLeft().whileTrue(Commands.runOnce(() -> m_ClimberSubsystem.leftJogDown()));
        m_driverController.povLeft().onFalse(Commands.runOnce(() -> m_ClimberSubsystem.stop()));
        
        m_driverController.povRight().whileTrue(Commands.runOnce(() -> m_ClimberSubsystem.rightJogDown()));
        m_driverController.povRight().onFalse(Commands.runOnce(() -> m_ClimberSubsystem.stop()));
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
