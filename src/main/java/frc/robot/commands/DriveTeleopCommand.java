package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import frc.robot.Constants.DriveConstants;
import java.util.function.*;

public class DriveTeleopCommand extends Command
{
    Drive m_drive;
    private final Supplier<Double> xSpeedGet, turnSpeedGet;
    private final SlewRateLimiter xLim, turnLim;

    public DriveTeleopCommand(Drive drive, Supplier<Double> xSpeedGet, Supplier<Double> turnSpeedGet)
    {
        this.xSpeedGet = xSpeedGet;
        this.turnSpeedGet = turnSpeedGet;
        this.m_drive = drive;

        this.xLim = new SlewRateLimiter(4*DriveConstants.kMaxAccel);
        this.turnLim = new SlewRateLimiter(4*DriveConstants.kMaxAccel);

        addRequirements(drive);
    }

    @Override
    public void initialize() { }


    @Override
    public void execute() 
    {

        //Grab joystick speeds from supplier
        double xSpeed = -1*xSpeedGet.get();
        double turnSpeed = -1*turnSpeedGet.get();
        
        //Deadband
        xSpeed = (Math.abs(xSpeed) < DriveConstants.kDriveControllerDeadband) ? 0 : xSpeed;
        turnSpeed = (Math.abs(turnSpeed) < DriveConstants.kDriveControllerDeadband) ? 0 : turnSpeed;

        //Scale speeds from [-1, 1] to [-maxSpeed, maxSpeed]
        xSpeed *= DriveConstants.kMaxSpeed;
        turnSpeed *= DriveConstants.kMaxAngularSpeed;

        //Slew rate. Limits the amount the robot can accelerate
        xSpeed = xLim.calculate(xSpeed);
        turnSpeed = turnLim.calculate(turnSpeed);

        //Call drive function
        m_drive.setDriveMotors(new ChassisSpeeds(xSpeed, 0, turnSpeed));

        
    }
    

    @Override
    public boolean isFinished() {return false;}

    @Override
    public void end(boolean interrupted) {}
}