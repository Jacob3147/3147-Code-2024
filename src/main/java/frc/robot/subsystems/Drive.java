package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
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



public class Drive extends SubsystemBase
{
    //Drive motors. CAN IDs are set in Constants
    private static final CANSparkMax frontLeft = new CANSparkMax(DriveConstants.frontLeft_Motor_Port, MotorType.kBrushless);
    private static final CANSparkMax backLeft = new CANSparkMax(DriveConstants.backLeft_Motor_Port, MotorType.kBrushless);
    private static final CANSparkMax frontRight = new CANSparkMax(DriveConstants.frontRight_Motor_Port, MotorType.kBrushless);
    private static final CANSparkMax backRight = new CANSparkMax(DriveConstants.backRight_Motor_Port, MotorType.kBrushless);
    
    //Get the encoders from Spark Max
    private static final RelativeEncoder leftEncoder = frontLeft.getEncoder();

    private static final RelativeEncoder rightEncoder = frontRight.getEncoder();

    //PID Controllers
    private final static PIDController leftPIDcontroller = new PIDController(DriveConstants.Kp_auto, DriveConstants.Ki_auto, DriveConstants.Kd_auto);
    private final static PIDController rightPIDcontroller = new PIDController(DriveConstants.Kp_auto, DriveConstants.Ki_auto, DriveConstants.Kd_auto);

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
        frontLeft.setIdleMode(IdleMode.kBrake);
        frontRight.restoreFactoryDefaults();
        frontRight.setIdleMode(IdleMode.kBrake);
        backLeft.restoreFactoryDefaults();
        backLeft.setIdleMode(IdleMode.kBrake);
        backRight.restoreFactoryDefaults();
        backRight.setIdleMode(IdleMode.kBrake);
        
        //Set the rear drives to follow the front drives
        backLeft.follow(frontLeft);
        backRight.follow(frontRight);                                                                                                    

        //Set the left side as inverted
        frontLeft.setInverted(true);

        //Initialize the pose estimator
        m_odometry = new DifferentialDrivePoseEstimator(
            m_kinematics, 
            navX.getRotation2d(),
            -getLeftEncoderPosition(), 
            -getRightEncoderPosition(),
            new Pose2d(new Translation2d(5, 5), new Rotation2d(0)),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(20))); 
        
        
        /* 
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
        }).start();*/
    
        //This is how PathPlanner will interact with the drivetrain
        AutoBuilder.configureLTV(
            this::poseSupplier, //Provides robot position on field
            this::poseSetter,   //Update robot position to a known position
            this::speedSupplier,//Provides robot speed
            this::setDriveMotors,//Drive robot based on chassis speeds,
            0.02,
            new ReplanningConfig(false, false),
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
        m_odometry.update(navX.getRotation2d(), -getLeftEncoderPosition(), -getRightEncoderPosition());
        field.setRobotPose(m_odometry.getEstimatedPosition());


        SmartDashboard.putNumber("odo x", m_odometry.getEstimatedPosition().getX());
        SmartDashboard.putNumber("odo y", m_odometry.getEstimatedPosition().getY());
        SmartDashboard.putNumber("odo t", m_odometry.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("Angle", navX.getYaw());
        SmartDashboard.putNumber("Left position", getLeftEncoderPosition());
        SmartDashboard.putNumber("Right position", getRightEncoderPosition());
        SmartDashboard.putNumber("Left velocity", getLeftEncoderVelocity());
        SmartDashboard.putNumber("Right velocity", getRightEncoderVelocity());

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


        frontLeft.setVoltage(leftFFoutput + leftPIDoutput);
        frontRight.setVoltage(rightFFoutput + rightPIDoutput);

        SmartDashboard.putNumber("Left target speed", target_left_velocity);
        SmartDashboard.putNumber("Right target speed", target_right_velocity);
    };


    Pose2d poseSupplier() { return m_odometry.getEstimatedPosition(); }
    void poseSetter(Pose2d p) {m_odometry.resetPosition(navX.getRotation2d(), -getLeftEncoderPosition(), -getRightEncoderPosition(), p); }
    ChassisSpeeds speedSupplier() { return m_kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(-getLeftEncoderVelocity(), -getRightEncoderVelocity())); }

    double getLeftEncoderPosition() { return leftEncoder.getPosition() * DriveConstants.kEncoderDistancePerPulse; }
    double getRightEncoderPosition() { return rightEncoder.getPosition() * DriveConstants.kEncoderDistancePerPulse; }
    double getLeftEncoderVelocity() { return leftEncoder.getVelocity() * DriveConstants.kEncoderDistancePerPulse / 60; }
    double getRightEncoderVelocity() { return rightEncoder.getVelocity() * DriveConstants.kEncoderDistancePerPulse / 60; }
    
    public static void setAutonPID() 
    { 
        leftPIDcontroller.setPID(DriveConstants.Kp_auto, DriveConstants.Ki_auto, DriveConstants.Kd_auto);
        rightPIDcontroller.setPID(DriveConstants.Kp_auto, DriveConstants.Ki_auto, DriveConstants.Kd_auto);
    }

    public static void setTeleopPID()
    {
        leftPIDcontroller.setPID(DriveConstants.Kp_tele, DriveConstants.Ki_tele, DriveConstants.Kd_tele);
        rightPIDcontroller.setPID(DriveConstants.Kp_tele, DriveConstants.Ki_tele, DriveConstants.Kd_tele);
    }


}
