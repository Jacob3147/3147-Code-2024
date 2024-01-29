// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



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
    
    public static final class DriveConstants
    {
        public static final int kDriverControllerPort = 0;

        public static final int frontLeft_Motor_Port = 2;
        public static final int backLeft_Motor_Port = 4;
        public static final int frontRight_Motor_Port = 3;
        public static final int backRight_Motor_Port = 1;

        //Drive physical properties
        public static final double kMaxSpeed = 5.2; // meters per second
        public static final double kMaxAccel = 3; // meters per second per second
        public static final double kMaxAngularSpeed = 2 * Math.PI; //rotations per second
        public static final double kTrackWidth = 0.6858; // meters
        public static final double kWheelDiameter = 0.1524; // meters
        public static final double gearReduction = 8.45;

        
        public static final int kEncoderResolution = 1;
        public static final double kEncoderDistancePerPulse = (kWheelDiameter * Math.PI) / (gearReduction * (double) kEncoderResolution);

        public static final double kDeadband = 0.05;

        //Drive feedforward constants
        public static final double Ks = 0.65;
        public static final double Kv = 2;
        
        //Drive feedback constants
        public static final double Kp = 0.5;
        public static final double Ki = 0;
        public static final double Kd = 0;
    }

}
