// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.TurretConstants;

/** Add your docs here. */
public class Turret extends SubsystemBase{
    private WPI_TalonSRX spinner;

    public Turret(){

        spinner = new WPI_TalonSRX(TurretConstants.kSpinner);


        //-----Turret Direction/spinner setup---
        //make sure spinner is powered off
        spinner.set(ControlMode.PercentOutput,0);

        //reset all confiurations (stops unexpected behavior)
        spinner.configFactoryDefault();

        //Set Neutral Mode
        spinner.setNeutralMode(NeutralMode.Brake);

        //Set Neutral Deadband
        spinner.configNeutralDeadband(0.1);

        //PID SETUP CONFIGURATION
        //configure Potenientometer (analog input) as PID feedback
        spinner.configSelectedFeedbackSensor(
            FeedbackDevice.Analog,
            PIDConstants.kPIDprimary,
            0
        );

        //configure sensor direciton
        spinner.setSensorPhase(false);
        spinner.setInverted(false);

        //assign PID values
        spinner.config_kP(PIDConstants.kSlot0,TurretConstants.kTurretGains.kP);
        spinner.config_kI(PIDConstants.kSlot0,TurretConstants.kTurretGains.kI);
        spinner.config_kD(PIDConstants.kSlot0,TurretConstants.kTurretGains.kD);

        //set closed loop period
        int closedLoopTimeMs = 1;
        spinner.configClosedLoopPeriod(PIDConstants.kSlot0, closedLoopTimeMs);

        //configure acceleration, cruise velocity, and ramp rate
        spinner.configMotionAcceleration(10); //* 100 * TurretConstants.kTicksPerDegree);
        spinner.configMotionCruiseVelocity(10); //* TurretConstants.kspinnerTargetSpeed * (TurretConstants.kTicksPerDegree ) / 10.0);
        spinner.configClosedloopRamp(0);
        
        //select the PID Slot to be used for primary PID loop
        spinner.selectProfileSlot(PIDConstants.kSlot0, PIDConstants.kPIDprimary);

        //enable soft limits
        //spinner.configForwardSoftLimitThreshold(TurretConstants.kMeasuredPosHorizontal + 71.0 * TurretConstants.kTicksPerDegree);
        //spinner.configReverseSoftLimitThreshold(TurretConstants.kMeasuredPosHorizontal + -12.0 * TurretConstants.kTicksPerDegree);
        spinner.setNeutralMode(NeutralMode.Brake);
        //spinner.configForwardSoftLimitEnable(true);
        // spinner.configReverseSoftLimitEnable(true);

        //set spinner percent output to 0
        spinner.set(ControlMode.PercentOutput, 0);
        
    }

    /**
   * Returns the position (in ticks) of the Turret
   */
    public double getSpinnerPosition() {
    // return spinner.getSelectedSensorPosition();
       return (spinner.getSelectedSensorPosition());
    }    

    public boolean isTurretWithinError(double targetPosition, double error) {
        // SmartDashboard.putNumber("erro1:",Math.abs(getspinnerPosition() - targetPosition));
        // SmartDashboard.putNumber("err2", targetPosition);
        return (Math.abs(getSpinnerPosition() - targetPosition) <= error);
    }

    /**
   * Aim the Turret using PID
   * @param degrees the target angle (in degrees) assuming horizontal is 0 and vertical is 90
   */
    public void setSpinnerPower(double power){
        spinner.set(ControlMode.PercentOutput, power);
    }

    public void setSpinnerPosition(double targetPosition){
        //set PID to run to a target Degrees
        spinner.set(ControlMode.MotionMagic,targetPosition);
    }

 

    /**
   * Stops motion of the rotating center spinner
   */
    public void stopspinner()
    {
        spinner.set(ControlMode.PercentOutput,0);
    }
}

