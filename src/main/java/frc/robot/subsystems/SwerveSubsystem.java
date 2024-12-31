// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OffsetConstants;
import frc.robot.Constants.SwerveMotorDeviceConstants;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new Subsystem. */

  private static final SwerveModule m_frontLeftModule = new SwerveModule(
          SwerveMotorDeviceConstants.kFrontLeftDriveMotorCANId,
          SwerveMotorDeviceConstants.kFrontLeftSteerMotorCANId,
          SwerveMotorDeviceConstants.kFrontLeftDriveMotorInverted,
          SwerveMotorDeviceConstants.kFrontLeftSteerMotorInverted,
          SwerveMotorDeviceConstants.kFrontLeftCANcoderCANId,
          SwerveMotorDeviceConstants.kFrontLeftCANcoderOffsetRad,
          SwerveMotorDeviceConstants.kFrontLeftCANcoderInverted);

  private static final SwerveModule m_frontRightModule = new SwerveModule(
          SwerveMotorDeviceConstants.kFrontRightDriveMotorCANId,
          SwerveMotorDeviceConstants.kFrontRightSteerMotorCANId,
          SwerveMotorDeviceConstants.kFrontRightDriveMotorInverted,
          SwerveMotorDeviceConstants.kFrontRightSteerMotorInverted,
          SwerveMotorDeviceConstants.kFrontRightCANcoderCANId,
          SwerveMotorDeviceConstants.kFrontRightCANcoderOffsetRad,
          SwerveMotorDeviceConstants.kFrontRightCANcoderInverted);

  private static final SwerveModule m_backLeftModule = new SwerveModule(
          SwerveMotorDeviceConstants.kBackLeftDriveMotorCANId,
          SwerveMotorDeviceConstants.kBackLeftSteerMotorCANId,
          SwerveMotorDeviceConstants.kBackLeftDriveMotorInverted,
          SwerveMotorDeviceConstants.kBackLeftSteerMotorInverted,
          SwerveMotorDeviceConstants.kBackLeftCANcoderCANId,
          SwerveMotorDeviceConstants.kBackLeftCANcoderOffsetRad,
          SwerveMotorDeviceConstants.kBackLeftCANcoderInverted);

  private static final SwerveModule m_backRightModule = new SwerveModule(
          SwerveMotorDeviceConstants.kBackRightDriveMotorCANId,
          SwerveMotorDeviceConstants.kBackRightSteerMotorCANId,
          SwerveMotorDeviceConstants.kBackRightDriveMotorInverted,
          SwerveMotorDeviceConstants.kBackRightSteerMotorInverted,
          SwerveMotorDeviceConstants.kBackRightCANcoderCANId,
          SwerveMotorDeviceConstants.kBackRightCANcoderOffsetRad,
          SwerveMotorDeviceConstants.kBackRightCANcoderInverted);

  private static Translation2d m_frontLeftLocation  = new Translation2d(+OffsetConstants.chasisXOffset, -OffsetConstants.chasisYOffset);
  private static Translation2d m_frontRightLocation = new Translation2d(+OffsetConstants.chasisXOffset, +OffsetConstants.chasisYOffset);
  private static Translation2d m_backLeftLocation   = new Translation2d(-OffsetConstants.chasisXOffset, -OffsetConstants.chasisYOffset);
  private static Translation2d m_backRightLocation  = new Translation2d(-OffsetConstants.chasisXOffset, +OffsetConstants.chasisYOffset);
  
  public static final SwerveDriveKinematics m_swerveDriveKinematics = new SwerveDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
  );

  private static final SwerveDriveOdometry m_odometer = new SwerveDriveOdometry(m_swerveDriveKinematics, new Rotation2d(0), 
    new SwerveModulePosition[] {
    m_frontLeftModule.getPosition(),
    m_frontRightModule.getPosition(),
    m_backLeftModule.getPosition(),
    m_backRightModule.getPosition()
  });

  private final StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();

  public SwerveSubsystem() {
    System.out.println("SwerveSubsystem: Ctor");
    
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    double [] currentState = getCurrentChasisSpeeds();
    publisher.set(new SwerveModuleState[] {
      new SwerveModuleState(currentState[1], Rotation2d.fromDegrees(currentState[0])),
      new SwerveModuleState(currentState[3], Rotation2d.fromDegrees(currentState[2])),
      new SwerveModuleState(currentState[5], Rotation2d.fromDegrees(currentState[4])),
      new SwerveModuleState(currentState[7], Rotation2d.fromDegrees(currentState[6]))
    });
  }

  public void setChasisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = m_swerveDriveKinematics.toSwerveModuleStates(speeds); 
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        m_frontLeftModule.setState(moduleStates[0]);
        m_frontRightModule.setState(moduleStates[1]);
        m_backLeftModule.setState(moduleStates[2]);
        m_backRightModule.setState(moduleStates[3]);

        showChasisSpeedsOnLogger("module");
    }

    public double[] getCurrentChasisSpeeds() {
      double loggingState[] = {m_frontLeftModule.getState().angle.getDegrees(),
        m_frontLeftModule.getState().speedMetersPerSecond,
        m_frontRightModule.getState().angle.getDegrees(),
        m_frontRightModule.getState().speedMetersPerSecond,
        m_backLeftModule.getState().angle.getDegrees(),
        m_backLeftModule.getState().speedMetersPerSecond,
        m_backRightModule.getState().angle.getDegrees(),
        m_backRightModule.getState().speedMetersPerSecond};

        return loggingState;
    }

    public void stopModules() {
        m_frontLeftModule.stop();
        m_frontRightModule.stop();
        m_backLeftModule.stop();
        m_backRightModule.stop();
    }

    public void showChasisSpeedsOnLogger(String labelString){
      double currentState[] = getCurrentChasisSpeeds();
      for ( int i = 0; i < currentState.length; i++) {
        String output = labelString + "State:" + Integer.toString(i);
        //SmartDashboard.putNumber("State:" + Integer.toString(i), currentState[i]);
        SmartDashboard.putNumber(output, currentState[i]);
        if ( Math.abs(currentState[i]) > 0.0 ) {
          if ( 1 == 0 ){
            System.out.print("State:[" + Integer.toString(i) + "]: " + currentState[i] + " / ");
            if (i == currentState.length - 1) {
              System.out.println();
            }
          }
        }
      }
    }

  public void moveMotors(String motorLabel, double speed){
    switch(motorLabel){
      case "FL":
        m_frontLeftModule.setSpeed(speed);
        break;
      case "FR":
        m_frontRightModule.setSpeed(speed);
        break;
      case "BL":
        m_backLeftModule.setSpeed(speed);
        break;
      case "BR":
        m_backRightModule.setSpeed(speed);
        break;
    }
  }

  public void moveMotorsByType(String motorType, double speed){
    m_frontLeftModule.setSpeed(motorType, speed);
    m_frontRightModule.setSpeed(motorType, speed);
    m_backLeftModule.setSpeed(motorType, speed);
    m_backRightModule.setSpeed(motorType, speed);
  
  }
}
