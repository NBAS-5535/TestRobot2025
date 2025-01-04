// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.SimpleTestCommand;
import frc.robot.commands.SwerveDriveComboMoveCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_swerveSubsystem.setDefaultCommand(new SwerveDriveCommand(m_swerveSubsystem, 
                                                      () -> -m_driverController.getRawAxis(OperatorConstants.kDriverYAxis),
                                                      () -> m_driverController.getRawAxis(OperatorConstants.kDriverXAxis),
                                                      () -> m_driverController.getRawAxis(OperatorConstants.kDriverRotAxis)
                                                      ));
    if (Constants.simtest) {
        m_driverController.b().onTrue(new SwerveDriveComboMoveCommand(m_swerveSubsystem, 0.25, 0., 0., 10.));
        m_driverController.a().onTrue(new SwerveDriveComboMoveCommand(m_swerveSubsystem, 0., 0., 0., 0.));
        m_driverController.x().onTrue(new SwerveDriveComboMoveCommand(m_swerveSubsystem, 0.25, 0., 0.3, 0.));
    } else {
        String label = "RF";
        m_driverController.x().whileTrue(new SimpleTestCommand(m_swerveSubsystem, label, 0.1)); 
        m_driverController.a().whileTrue(new SimpleTestCommand(m_swerveSubsystem, "drive"));
        m_driverController.b().whileTrue(new SimpleTestCommand(m_swerveSubsystem, "steer")); 
        m_driverController.y().onTrue(new SimpleTestCommand(m_swerveSubsystem, 0.));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.autoDriveForwardByTime(m_swerveSubsystem);
  }
}
