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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;


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


        m_drive = new DifferentialDrive(frontLeft, frontRight);
        
        //Set the rear drives to follow the front drives
        backLeft.follow(frontLeft);
        backRight.follow(frontRight);                                                                                                    

        //Set the left side as inverted
        frontLeft.setInverted(true);

        //Initialize the pose estimator
        m_odometry = new DifferentialDrivePoseEstimator(
            m_kinematics, 
            navX.getRotation2d(),
            getLeftEncoderPosition(), 
            getRightEncoderPosition(),
            new Pose2d(new Translation2d(0, 0), new Rotation2d(0))); 
        
        
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
            this::poseSupplier,
            this::poseSetter,
            this::speedSupplier,
            this::setDriveMotors,
            new ReplanningConfig(false, true),
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
    
    //I stole this code from an example online. In theory we run it to get a log file, and then put it into a tool called SysID
    //to tell us exactly what the PID and feedforward should be
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) -> {
                frontLeft.setVoltage(volts.in(Volts));
                frontRight.setVoltage(volts.in(Volts));
              },
              log -> {
                log.motor("drive-left")
                    .voltage( m_appliedVoltage.mut_replace( frontLeft.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition( m_distance.mut_replace(getLeftEncoderPosition(), Meters))
                    .linearVelocity( m_velocity.mut_replace(getLeftEncoderVelocity(), MetersPerSecond));

                log.motor("drive-right")
                    .voltage( m_appliedVoltage.mut_replace( frontRight.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(getRightEncoderPosition(), Meters))
                    .linearVelocity( m_velocity.mut_replace(getRightEncoderVelocity(), MetersPerSecond));
              },
              this));
    
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) 
    {
    return m_sysIdRoutine.quasistatic(direction);
    }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) 
  {
    return m_sysIdRoutine.dynamic(direction);
    }


    //Constantly update the odometry with the gyro and encoders. Update the dashboard
    public void periodic() 
    {

        m_odometry.update(navX.getRotation2d(), -getLeftEncoderPosition(), -getRightEncoderPosition());
        SmartDashboard.putNumber("odo x", m_odometry.getEstimatedPosition().getX());
        SmartDashboard.putNumber("odo y", m_odometry.getEstimatedPosition().getY());
        SmartDashboard.putNumber("odo t", m_odometry.getEstimatedPosition().getRotation().getDegrees());

        field.setRobotPose(m_odometry.getEstimatedPosition());
        SmartDashboard.putNumber("Angle", navX.getYaw());
        SmartDashboard.putNumber("Left position", getLeftEncoderPosition());
        SmartDashboard.putNumber("Right position", getRightEncoderPosition());
        SmartDashboard.putNumber("Left velocity", getLeftEncoderVelocity());
        SmartDashboard.putNumber("Right velocity", getRightEncoderVelocity());

        SmartDashboard.putNumber("left voltage", frontLeft.getAppliedOutput());
        SmartDashboard.putNumber("right voltage", frontRight.getAppliedOutput());

    }

    //Controls the robot using chassis speeds
    public void setDriveMotors(ChassisSpeeds speeds)
    {

        //Convert the chassis speeds to wheel speeds
        var wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);

        double target_left_velocity = -1*wheelSpeeds.leftMetersPerSecond;
        double target_right_velocity = -1*wheelSpeeds.rightMetersPerSecond;

        var leftFFoutput = leftFeedforward.calculate(target_left_velocity);
        var rightFFoutput = rightFeedforward.calculate(target_right_velocity);

        double leftPIDoutput = leftPIDcontroller.calculate(getLeftEncoderVelocity(), target_left_velocity);
        double rightPIDoutput = rightPIDcontroller.calculate(getRightEncoderVelocity(), target_right_velocity);

        frontLeft.setVoltage(leftFFoutput + leftPIDoutput);
        frontRight.setVoltage(rightFFoutput + rightPIDoutput);

        m_drive.feed();

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
