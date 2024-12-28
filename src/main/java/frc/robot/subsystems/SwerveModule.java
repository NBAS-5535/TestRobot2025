// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;

/** Add your docs here. */
public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;

    private final PIDController steerPidController;

    private SwerveModuleState currentState;

    public SwerveModule(int driveMotorCANId, int steerMotorCANId,
                        boolean driveMotorInverted, boolean steerMotorInverted,
                        int absoluteEncoderCANid, double absoluteEncoderOffsetRad, boolean absoluteEncoderInverted) {

        driveMotor = new CANSparkMax(driveMotorCANId, MotorType.kBrushless);
        steerMotor = new CANSparkMax(steerMotorCANId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorInverted);
        steerMotor.setInverted(steerMotorInverted);

        steerPidController = new PIDController(ModuleConstants.kPSteer, 0., 0.);
        steerPidController.enableContinuousInput(-Math.PI, Math.PI);

        currentState = new SwerveModuleState();

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
}
