// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ModuleConstants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** Add your docs here. */
public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;

    private final PIDController steerPidController;

    private SwerveModuleState currentState;

    public SwerveModule(int driveMotorCANId, int steerMotorCANId,
                        boolean driveMotorInverted, boolean steerMotorInverted,
                        int absoluteEncoderCANid, double absoluteEncoderOffsetRad, boolean absoluteEncoderInverted) {

        driveMotor = new CANSparkMax(driveMotorCANId, MotorType.kBrushless);
        steerMotor = new CANSparkMax(steerMotorCANId, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        steerMotor.restoreFactoryDefaults();
        
        driveMotor.setInverted(driveMotorInverted);
        steerMotor.setInverted(steerMotorInverted);

        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();
            
        steerPidController = new PIDController(ModuleConstants.kPSteer, 0., 0.);
        steerPidController.enableContinuousInput(-Math.PI, Math.PI);

        currentState = new SwerveModuleState(0., Rotation2d.fromDegrees(0.));

        resetEncoders();

        System.out.println("SwerveModule:" + String.valueOf(driveMotorCANId) + " - " + String.valueOf(steerMotorCANId));

    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return steerEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return steerEncoder.getVelocity();
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        steerEncoder.setPosition(0.);
    }

    public SwerveModuleState getState1() {
        //Rotation2d angle = new Rotation2d(getTurningPosition());
        //currentState = SwerveModuleState(getDriveVelocity(), angle);
        return currentState;
    }

    public SwerveModuleState getState(){
        return currentState;
    }


    public void setState(SwerveModuleState newState){
        currentState = newState;
    }
  
    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }

    public void setSpeed(double speed) {
        driveMotor.set(speed);
        steerMotor.set(speed);
    }

    public void setSpeed(String motorType, double speed){
        switch ( motorType) {
            case "drive":
                driveMotor.set(speed);
                break;
            case "steer":
                steerMotor.set(speed);
                break;
        }
    }
}
