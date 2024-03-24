package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.Supplier;

import javax.lang.model.util.ElementScanner14;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LimelightConstants;
import frc.robot.utility.Vision;



public class Drive extends SubsystemBase
{
    //Drive motors. CAN IDs are set in Constants
    private static final CANSparkMax frontLeft = new CANSparkMax(frontLeft_Motor_Port, MotorType.kBrushless);
    private static final CANSparkMax backLeft = new CANSparkMax(backLeft_Motor_Port, MotorType.kBrushless);
    private static final CANSparkMax frontRight = new CANSparkMax(backRight_Motor_Port, MotorType.kBrushless);
    private static final CANSparkMax backRight = new CANSparkMax(frontRight_Motor_Port, MotorType.kBrushless);
    
    //Get the encoders from Spark Max
    private static final RelativeEncoder leftEncoder = frontLeft.getEncoder();

    private static final RelativeEncoder rightEncoder = frontRight.getEncoder();

    //PID Controllers
    private final static PIDController leftPIDcontroller = new PIDController(Kp_auto, Ki_auto, Kd_auto);
    private final static PIDController rightPIDcontroller = new PIDController(Kp_auto, Ki_auto, Kd_auto);
    private final static PIDController anglePID = new PIDController(0.1,0.02,0);

    //Feedforward Controllers
    private final SimpleMotorFeedforward leftFeedforward = new SimpleMotorFeedforward(Ks, Kv);
    private final SimpleMotorFeedforward rightFeedforward = new SimpleMotorFeedforward(Ks, Kv);

    Supplier<Rotation2d> rotationSupplier;

    //Kinematics object converts between Chassis Speeds (x, y, angle) and wheel speeds (left wheels, right wheels)
    //Since we have tank drive, y = 0 because we can't move sideways
    private static final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);

    //Pose Estimator combines encoders, gyro, and optionally limelight to give a Pose2d
    //Pose2d is a combination of a Rotation2d and a Translation2d
    public static DifferentialDrivePoseEstimator m_odometry;

    private Field2d field = new Field2d();
    
    boolean LL_front_has_pose = false;
    boolean LL_rear_has_pose = false;
    
    Pose2d LL_rear_pose;

    //Constructor
    public Drive(Supplier<Rotation2d> rotationSupplier)
    {
        this.rotationSupplier = rotationSupplier;
        
        frontLeft.restoreFactoryDefaults();
        frontLeft.setIdleMode(IdleMode.kBrake);
        frontRight.restoreFactoryDefaults();
        frontRight.setIdleMode(IdleMode.kBrake);
        backLeft.restoreFactoryDefaults();
        backLeft.setIdleMode(IdleMode.kCoast);
        backRight.restoreFactoryDefaults();
        backRight.setIdleMode(IdleMode.kCoast);

        frontLeft.setSmartCurrentLimit(40);
        frontRight.setSmartCurrentLimit(40);
        backLeft.setSmartCurrentLimit(40);
        backRight.setSmartCurrentLimit(40);

        //Set the rear drives to follow the front drives
        backLeft.follow(frontLeft);
        backRight.follow(frontRight);                                                                                                    

        //Set the left side as inverted
        frontLeft.setInverted(true);

        //Initialize the pose estimator
        m_odometry = new DifferentialDrivePoseEstimator(
            m_kinematics, 
            rotationSupplier.get(),
            -getLeftEncoderPosition(), 
            -getRightEncoderPosition(),
            new Pose2d(new Translation2d(1.37, 5.55), new Rotation2d(Math.PI)),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10))); 
        
    
        //This is how PathPlanner will interact with the drivetrain
        AutoBuilder.configureLTV(
            this::poseSupplier, //Provides robot position on field
            this::poseSetter,   //Update robot position to a known position
            this::speedSupplier,//Provides robot speed
            this::setDriveMotors,//Drive robot based on chassis speeds,
            0.02,
            new ReplanningConfig(true, true),
            () -> { //mini function that returns true on red alliance
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this);
        
        //put the trajectory onto the fake field
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
        SmartDashboard.putData("Field", field);
    }



    //Constantly update the odometry with the gyro and encoders. Update the dashboard
    public void periodic() 
    {
        SmartDashboard.putNumber("Gyro", rotationSupplier.get().getDegrees());
        SmartDashboard.putBoolean("LL Front valid?", LL_front_has_pose);
        SmartDashboard.putBoolean("LL Rear valid?", LL_rear_has_pose);
        
        if(DriverStation.isTeleop())
        {
            LL_front_has_pose = Vision.EvaluateLimelightNew(LimelightConstants.limelight_1_name, field);
            LL_rear_has_pose = Vision.EvaluateLimelightNew(LimelightConstants.limelight_2_name, field);
        }

        m_odometry.update(rotationSupplier.get(), -getLeftEncoderPosition(), -getRightEncoderPosition());
        field.setRobotPose(m_odometry.getEstimatedPosition());

        
        SmartDashboard.putNumber("odo x", m_odometry.getEstimatedPosition().getX());
        SmartDashboard.putNumber("odo y", m_odometry.getEstimatedPosition().getY());
        SmartDashboard.putNumber("odo angle", m_odometry.getEstimatedPosition().getRotation().getDegrees());

    }

    //Controls the robot using chassis speeds
    public void setDriveMotors(ChassisSpeeds speeds)
    {
        //Convert the chassis speeds to wheel speeds
        DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);

        double target_left_velocity = -1*wheelSpeeds.leftMetersPerSecond;
        double target_right_velocity = -1*wheelSpeeds.rightMetersPerSecond;

        
        double leftFFoutput = leftFeedforward.calculate(target_left_velocity);
        double rightFFoutput = rightFeedforward.calculate(target_right_velocity);

        double leftPIDoutput = leftPIDcontroller.calculate(getLeftEncoderVelocity(), target_left_velocity);
        double rightPIDoutput = rightPIDcontroller.calculate(getRightEncoderVelocity(), target_right_velocity);

        SmartDashboard.putNumber("target left", target_left_velocity);
        SmartDashboard.putNumber("target right", target_right_velocity);
        frontLeft.setVoltage(leftFFoutput + leftPIDoutput);
        frontRight.setVoltage(rightFFoutput + rightPIDoutput);
    };

    ChassisSpeeds turnToAngle_speeds = new ChassisSpeeds(0,0,0);
    public boolean turnToAngle(double angle)
    {
        anglePID.enableContinuousInput(-180,180);
        anglePID.setTolerance(4);
        
        anglePID.setIZone(10);

        double turnRate = anglePID.calculate(poseSupplier().getRotation().getDegrees(), angle);
        turnRate = turnRate > kMaxAngularSpeed ? kMaxAngularSpeed : turnRate;

        turnToAngle_speeds.omegaRadiansPerSecond = turnRate;
        setDriveMotors(turnToAngle_speeds);

        return anglePID.atSetpoint();
    }

    public Pose2d poseSupplier() { return m_odometry.getEstimatedPosition(); }

    void poseSetter(Pose2d p) {m_odometry.resetPosition(rotationSupplier.get(), -getLeftEncoderPosition(), -getRightEncoderPosition(), p); }

    DifferentialDriveWheelSpeeds supplier_wheel_speeds = new DifferentialDriveWheelSpeeds();
    ChassisSpeeds speedSupplier() 
    {
        supplier_wheel_speeds.leftMetersPerSecond = -getLeftEncoderVelocity();
        supplier_wheel_speeds.rightMetersPerSecond = -getRightEncoderVelocity();
        return m_kinematics.toChassisSpeeds(supplier_wheel_speeds); 
    }

    double getLeftEncoderPosition() { return leftEncoder.getPosition() * kEncoderDistancePerPulse; }
    double getRightEncoderPosition() { return rightEncoder.getPosition() * kEncoderDistancePerPulse; }
    double getLeftEncoderVelocity() { return leftEncoder.getVelocity() * kEncoderDistancePerPulse / 60; }
    double getRightEncoderVelocity() { return rightEncoder.getVelocity() * kEncoderDistancePerPulse / 60; }
    
    public static void setAutonPID() 
    { 
        leftPIDcontroller.setPID(Kp_auto, Ki_auto, Kd_auto);
        rightPIDcontroller.setPID(Kp_auto, Ki_auto, Kd_auto);
    }

    public static void setTeleopPID()
    {
        leftPIDcontroller.setPID(Kp_tele, Ki_tele, Kd_tele);
        rightPIDcontroller.setPID(Kp_tele, Ki_tele, Kd_tele);
    }

    public static double DistanceToSpeaker()
    {
        Pose2d currentPose = m_odometry.getEstimatedPosition();
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();

        double xToSpeaker;
        double yToSpeaker;
        var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    if (alliance.get() == DriverStation.Alliance.Blue)
                    {
                        yToSpeaker = blue_speaker_y - currentY;
                        xToSpeaker = blue_speaker_x - currentX;
                    }
                    else
                    {
                        yToSpeaker = red_speaker_y - currentY;
                        xToSpeaker = red_speaker_x - currentX;
                    }
                }
                else
                {
                    xToSpeaker = 1;
                    yToSpeaker = 0;
                }

                
        return Math.sqrt(xToSpeaker*xToSpeaker + yToSpeaker*yToSpeaker);
    }

    public static double AngleToSpeaker()
    {
        Pose2d currentPose = m_odometry.getEstimatedPosition();
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();

        double xToSpeaker;
        double yToSpeaker;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Blue)
            {
                yToSpeaker = blue_speaker_y - currentY;
                xToSpeaker = blue_speaker_x - currentX;
                return Units.radiansToDegrees(Math.atan2(yToSpeaker, xToSpeaker));
            }        
            else
            {
                yToSpeaker = red_speaker_y - currentY;
                xToSpeaker = red_speaker_x - currentX;
                return Units.radiansToDegrees(Math.atan2(yToSpeaker, xToSpeaker));
            }
        }
        else return 0;
        
    }
    

}
