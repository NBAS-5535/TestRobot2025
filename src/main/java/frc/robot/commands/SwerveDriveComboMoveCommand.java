// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDriveComboMoveCommand extends Command {
  /** Creates a new SwerveDriveComboMoveCommand. */
  private final SwerveSubsystem m_swerve;

  //private final double forwardSpeed, leftSpeed, turnSpeed;
  private ChassisSpeeds m_chasisSpeeds;

  private double timeToStop;

  public SwerveDriveComboMoveCommand(SwerveSubsystem swerve, double forwardSpeed,
                                   double leftSpeed, double turnSpeed, double timer) {

    this.m_swerve = swerve;
    //this.forwardSpeed = forwardSpeed;
    //this.leftSpeed = leftSpeed;
    //this.turnSpeed = turnSpeed;

    this.m_chasisSpeeds = new ChassisSpeeds(forwardSpeed, leftSpeed, turnSpeed);

    // check if there is a timer set
    timeToStop = 0.;
    if ( timer > 0.) {
      timeToStop = Timer.getFPGATimestamp() + timer;
    }
    SmartDashboard.putNumber("timeToStop", timeToStop);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(getName() + " started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerve.showChasisSpeedsOnLogger("execute");
    m_swerve.setChasisSpeeds(m_chasisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(getName() + " interrupted!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ( timeToStop > 0.) {
      System.out.println(getName() + " not finished!");
      return isTimedOut();
    } else {
      System.out.println(getName() + " finished!");
      return false;
    }
  }

  private boolean isTimedOut(){
    return Timer.getFPGATimestamp() >= timeToStop;
  }
}
