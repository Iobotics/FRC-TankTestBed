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
public final class Constants {
    public static final class RobotMap{     
        
        // Drivetrain devices (motors) 
        public static final int kLeftMaster = 1;
        public static final int kLeftSlave = 2;
        public static final int kRightMaster = 3;
        public static final int kRightSlave = 4;
    }
    
    public static final class OIConstants{
        public static final int kJoystick1 = 1;
        public static final int kJoystick2 = 0;
        public static final int kXbox1 = 2;
    }
}