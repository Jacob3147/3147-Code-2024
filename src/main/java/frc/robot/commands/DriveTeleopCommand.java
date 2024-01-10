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

        this.xLim = new SlewRateLimiter(2*DriveConstants.kMaxAccel);
        this.turnLim = new SlewRateLimiter(2*DriveConstants.kMaxAccel);

        addRequirements(drive);
    }

    @Override
    public void initialize() { }


    @Override
    public void execute() 
    {
        //Grab joystick speeds from supplier
        //Scale speeds from [-1, 1] to [-maxSpeed, maxSpeed]
        double xSpeed = DriveConstants.kMaxSpeed*xSpeedGet.get();
        double turnSpeed = DriveConstants.kMaxAngularSpeed*turnSpeedGet.get();

        
        //Slew rate. Limits the amount the robot can accelerate
        xSpeed = xLim.calculate(xSpeed);
        turnSpeed = turnLim.calculate(turnSpeed);


        //Call drive function
        m_drive.setDriveMotors.accept(new ChassisSpeeds(xSpeed, 0, turnSpeed));

        super.execute();
    }
    

    @Override
    public boolean isFinished() {return false;}

    @Override
    public void end(boolean interrupted) {}
}