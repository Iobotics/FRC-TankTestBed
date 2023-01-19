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
        public static final int kRightMaster = 4;
        public static final int kRightSlave = 3;
    }

    public static final class OIConstants{
        public static final int kJoystick1 = 1;
        public static final int kJoystick2 = 0;
        public static final int kXbox1 = 2;
    }

    public static final class TurretConstants{
        public static final int kSpinner = 5;

         //for the TalonSRX, a Error * kP of 1023 is considered "full". For our poteniometer, the typical range of motion is ~300-500, meaning
        //the greatest error we will likely get is of 200, with the error more likely to be closer to 100 or 50 in a given circumstance.
        //More info here: https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#position-closed-loop-control-mode
        //good recomendatation for smooth control: Start "D" term ~10x to 100x P term to smoothen movement, and if not reacing target, Integral term should start .01x P gain
        //in this case, 50  * our error of ~ 200  at a given point is going to be 400, wwhich is only about 1/2 full power, but is still enough. If our netural
        //deadband is .04 of full power, which coriponds to an error of aproximately 40 encoder units, or 16 degrees. From experience, the arm begins
        //to oscilate at ~100 units, which would mean an error of aproximately 8 degrees at the current neautral deadband.
        /* 	                                    			  kP   kI   kD   kF   Iz  PeakOut */
        public static final Gains kTurretGains = new Gains( 30.0, 0.0,  30.0, 0.0, 0,  1 ); //ARM PID values
        public static final double kTicksPerDegree = (1023.0/10.0) * (170.0 /30.0) * (30.0/20.0) * (1./360.); 
        public static final int kMeasuredPosHorizontal = -260;
        public static final double kMaxGravityFF = .12;
        //Target speed is in Degrees/second
        public static final double kArmTargetSpeed = 40;


    }

    public static final class Pneumatics{
        public static final int kDoubleSolenoidLeftSlot = 0;       
        public static final int kDoubleSolenoidRightSlot = 0;
    }

    public static final class PIDConstants{
        public static final int kPIDprimary = 0;
        public static final int kPIDturn = 1;
        public static final int kRemoteFilter0 = 0;
        public static final int kRemoteFilter1 = 1;
        public final static int kSlot0 = 0;
        public final static int kSlot1 = 1;
    }
}
