package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Drive extends SubsystemBase
{
    //Drive motors. CAN IDs are set in Constants
    private static final CANSparkMax frontLeft = new CANSparkMax(DriveConstants.frontLeft_Motor_Port, MotorType.kBrushless);
    private static final CANSparkMax backLeft = new CANSparkMax(DriveConstants.backLeft_Motor_Port, MotorType.kBrushless);
    private static final CANSparkMax frontRight = new CANSparkMax(DriveConstants.frontRight_Motor_Port, MotorType.kBrushless);
    private static final CANSparkMax backRight = new CANSparkMax(DriveConstants.backRight_Motor_Port, MotorType.kBrushless);
    

    //Tank drive object
    DifferentialDrive m_drive;
    
    //Get the encoders from Spark Max
    private static final RelativeEncoder leftEncoder = frontLeft.getEncoder();
    private static final RelativeEncoder rightEncoder = frontRight.getEncoder();

    //PID Controllers
    private final PIDController leftPIDcontroller = new PIDController(DriveConstants.Kp, 0, 0);
    private final PIDController rightPIDcontroller = new PIDController(DriveConstants.Kp, 0, 0);

    //Feedforward Controllers
    private final SimpleMotorFeedforward leftFeedforward = new SimpleMotorFeedforward(DriveConstants.Ks, DriveConstants.Kv);
    private final SimpleMotorFeedforward rightFeedforward = new SimpleMotorFeedforward(DriveConstants.Ks, DriveConstants.Kv);

    //Gyro
    private static final AHRS navX = new AHRS();

    //Kinematics object converts between Chassis Speeds (x, y, angle) and wheel speeds (left wheels, right wheels)
    //Since we have tank drive, y = 0 because we can't move sideways
    private static final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackWidth);

    //Pose Estimator combines encoders, gyro, and optionally limelight to give a Pose2d
    //Pose2d is a combination of a Rotation2d and a Translation2d
    public static DifferentialDrivePoseEstimator m_odometry;


    private Field2d field = new Field2d();

    //Constructor
    public Drive()
    {
        frontLeft.restoreFactoryDefaults();
        frontLeft.setIdleMode(IdleMode.kCoast);
        frontRight.restoreFactoryDefaults();
        frontRight.setIdleMode(IdleMode.kCoast);
        backLeft.restoreFactoryDefaults();
        backLeft.setIdleMode(IdleMode.kCoast);
        backRight.restoreFactoryDefaults();
        backRight.setIdleMode(IdleMode.kCoast);


        m_drive = new DifferentialDrive(frontLeft, frontRight);
        
        //Set the rear drives to follow the front drives
        backLeft.follow(frontLeft);
        backRight.follow(frontRight);                                                                                                    

        //Set the left side as inverted
        frontRight.setInverted(true);

        //Initialize the pose estimator
        m_odometry = new DifferentialDrivePoseEstimator(
            m_kinematics, 
            navX.getRotation2d(),
            leftEncoder.getPosition(), 
            -rightEncoder.getPosition(),
            new Pose2d(new Translation2d(0, 0), new Rotation2d(0))); 
        
        //Tell the encoders how many ticks (42 ticks per Neo rotation) equals a meter
        leftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
        rightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);

        leftEncoder.setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse/60);
        rightEncoder.setVelocityConversionFactor(DriveConstants.kEncoderDistancePerPulse/60);
        
        //Zero the encoders
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);

        //Reset gyro on a delay. Thread created so nothing waits on it. Delay because this constructor will be called while gyro still starting up.
        new Thread(() -> {
            try 
            {
                Thread.sleep(1000);
                navX.reset();
            } catch(Exception e) {}
        }).start();
    
        //This is how PathPlanner will interact with the drivetrain
        AutoBuilder.configureRamsete(
            poseSupplier,
            poseSetter,
            speedSupplier,
            setDriveMotors,
            new ReplanningConfig(),
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );

        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
        SmartDashboard.putData("Field", field);
    }

    //Constantly update the odometry with the gyro and encoders. Update the dashboard
    public void periodic() 
    {
        m_odometry.update(navX.getRotation2d(), leftEncoder.getPosition(), -1*rightEncoder.getPosition());

        SmartDashboard.putNumber("Angle", navX.getYaw());
        SmartDashboard.putNumber("Left position", leftEncoder.getPosition());
        SmartDashboard.putNumber("Right position", -1*rightEncoder.getPosition());
        SmartDashboard.putNumber("Left velocity", leftEncoder.getVelocity());
        SmartDashboard.putNumber("Right velocity", -1*rightEncoder.getVelocity());
    }


    //This is a special pattern called a Consumer used in "functional programming".
    //You create it with Consumer<Type> name = (variable of type) -> {function body}

    //Controls the robot using chassis speeds
    public Consumer<ChassisSpeeds> setDriveMotors = (chassisSpeeds) -> {

        //Convert the chassis speeds to wheel speeds
        var wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);

        double target_left_velocity = wheelSpeeds.leftMetersPerSecond;
        double target_right_velocity = wheelSpeeds.rightMetersPerSecond;

        var leftFFoutput = leftFeedforward.calculate(target_left_velocity);
        var rightFFoutput = rightFeedforward.calculate(target_right_velocity);

        double leftPIDoutput = leftPIDcontroller.calculate(leftEncoder.getVelocity(), target_left_velocity);
        double rightPIDoutput = rightPIDcontroller.calculate(-rightEncoder.getVelocity(), target_right_velocity);

        frontLeft.setVoltage(leftFFoutput + leftPIDoutput);
        frontRight.setVoltage(rightFFoutput + rightPIDoutput);

        m_drive.feed();

        SmartDashboard.putNumber("Left target speed", target_left_velocity);
        SmartDashboard.putNumber("Right target speed", target_right_velocity);
    };

    //This is a special pattern called a Supplier used in "functional programming"
    //You create it with Supplier<Type> name = () -> {return value}
    Supplier<Pose2d> poseSupplier = () -> m_odometry.getEstimatedPosition();
    Consumer<Pose2d> poseSetter = (p) -> m_odometry.resetPosition(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), p);
    Supplier<ChassisSpeeds> speedSupplier = () -> new ChassisSpeeds(leftEncoder.getVelocity(), 0, navX.getRate()*Constants.degrees_to_radians);

    
    

        
}
