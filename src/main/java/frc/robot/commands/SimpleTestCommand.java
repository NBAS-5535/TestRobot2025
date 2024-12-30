// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SimpleTestCommand extends Command {
  /** Creates a new SimpleTestCommand. */
  private final SwerveSubsystem m_swerve;
  private String m_label;
  private double m_speed = 0.1;

  private String m_commandType;

  public SimpleTestCommand(SwerveSubsystem swerve, String label, double speed) {
    this.m_swerve = swerve;
    this.m_label = label;
    this.m_speed = speed;
    this.m_commandType = "motorLocation";
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public SimpleTestCommand(SwerveSubsystem swerve, String motorType){
    this.m_swerve = swerve;
    this.m_label = motorType;
    this.m_commandType = "motorType";
  }

  public SimpleTestCommand(SwerveSubsystem swerve, double speed) {
    this.m_swerve = swerve;
    this.m_label = "";
    this.m_speed = 0.;
    this.m_commandType = "stop";
  }
    // Use addRequirements() here to declare subsystem dependencies.
    
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_commandType) {
      case "motorType":
        m_swerve.moveMotorsByType(m_label, m_speed);
        break;
      case "motorLocation":
        m_swerve.moveMotors(m_label, m_speed);
        break;
      case "stop":
        m_swerve.stopModules();
        break;
      default:
        break;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
