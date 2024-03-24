// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
    public static final double degrees_to_radians = Math.PI / 180;
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    
    public static final double amptrap_handoff_shooter_speed = 0.03;
    public static final double amptrap_handoff_feed_time = 0.3;
    public static final double speaker_feed_time = 1;
    public static final double amp_deposit_speed = 0.4;
    public static final double trap_deposit_speed = 1;
    public static final double amptrap_deposit_time = 1;
    public static final double rumble_time = 0.5;
    public static final double rumble_intensity = 0.5;

    public static final class DriveConstants
    {
        public static final int frontLeft_Motor_Port = 1;
        public static final int backLeft_Motor_Port = 2;
        public static final int frontRight_Motor_Port = 3;
        public static final int backRight_Motor_Port = 4;

        //Drive physical properties
        public static final double kMaxSpeed = 5.2; // meters per second
        public static final double kMaxAccel = 3; // meters per second per second
        public static final double kMaxAngularSpeed = 2 * Math.PI; //rotations per second
        public static final double kTrackWidth = 0.5588; // meters
        public static final double kWheelDiameter = 0.1524; // meters
        public static final double gearReduction = 8.45;

        public static final double drive_motor_current_limit = 40;
        public static final int kEncoderResolution = 1;
        public static final double kEncoderDistancePerPulse = (kWheelDiameter * Math.PI) / (gearReduction * (double) kEncoderResolution);

        public static final double kDriveControllerDeadband = 0.08;

        //Drive feedforward constants
        public static final double Ks = 0.5;
        public static final double Kv = 1.8;
        
        //Drive feedback constants
        public static final double Kp_auto = 1; //1 //PID
        public static final double Ki_auto = 0;
        public static final double Kd_auto = 0; //0

        public static final double Kp_tele = 1; 
        public static final double Ki_tele = 0;
        public static final double Kd_tele = 0; //0.1

        public static final double blue_speaker_x = 0;
        public static final double red_speaker_x = 16.46;
        public static final double blue_speaker_y = 5.5;
        public static final double red_speaker_y = 5.5;
    }

    public static final class IntakeConstants
    {
        public static final int intake_motor_port = 8;
        public static final int intake_solenoid_port_a = 15;
        public static final int intake_solenoid_port_b = 0;
        public static final int intake_sensor_port = 1;
        public static final double intake_fwd_speed = 1;
        public static final double intake_reverse_speed = -0.5;
    }

    public static final class ShooterConstants
    {
        public static final int top_motor_port = 5;
        public static final int bottom_motor_port = 7;
        public static final int tilt_moter_port = 6;
        public static final int stage_1_port_a = 12;
        public static final int stage_1_port_b = 9;
        public static final int stage_2_port_a = 11;
        public static final int stage_2_port_b = 10;
        public static final int encoder_port = 0;

        public static final int tilt_offset = -94;

        public static final double Ks = 0.05;
        public static final double Kg = 0.33;
        public static final double Kv = 1.6;
        public static final double Ka = 0.01;
        public static final double Kp = 4;
        public static final double Ki = 0;
        public static final double Kd = 0;
        public static final double maxV = 1;
        public static final double maxA = 1;

        public static final double tilt_angle_amp = 30;
        public static final double tilt_angle_trap = -59; //abs(angle) < 35 breaks height
        public static final double tilt_angle_rest = -50;

        public static final double shot_power = 1;

        public static final double distance_1 = 1.3;
        public static final double angle_1 = -48;

        public static final double distance_2 = 1.5;
        public static final double angle_2 = -43;

        public static final double distance_3 = 2;
        public static final double angle_3 = -32;

        public static final double distance_4 = 2.5;
        public static final double angle_4 = -26;

        public static final double distance_5 = 3;
        public static final double angle_5 = -20;

        public static final double distance_6 = 3.5;
        public static final double angle_6 = -17;

        public static final double distance_7 = 4;
        public static final double angle_7 = -15;

        public static final double distance_8 = 4.5;
        public static final double angle_8 = -12;

        public static final double distance_9 = 5;
        public static final double angle_9 = -9;
    }

    public static final class LimelightConstants
    {
        public static final String limelight_1_name = "limelight-front";
        public static final String limelight_2_name = "limelight-rear";

    }

    public static final class ClimbConstants
    {
        public static final int left_climb_port = 10;
        public static final int right_climb_port = 9;

        public static final double climber_rotations_per_mm = 1;
        public static final int left_limit_port = 2;
        public static final int right_limit_port = 3;

        public static final double climbSpeed = 0.4;
    }
}
