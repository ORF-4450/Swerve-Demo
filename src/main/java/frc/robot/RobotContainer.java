// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.SwerveDriveBase;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
  // The robot's subsystems and commands are defined here.
  public final SwerveDriveBase m_drivetrainSubsystem = new SwerveDriveBase();

  private final XboxController m_controller = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() 
  {
    Util.consoleLog();

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement (throttle)
    // Left stick X axis -> left and right movement (strafe)
    // Right stick X axis -> rotation
    // Note: X and Y axis on stick is opposite X and Y axis on the WheelSpeeds object.
    // Wheelspeeds X axis is + down the field away from alliance wall. +Y axis is left
    // when standing at alliance wall looking down the field.
    // This is handled here by swapping the inputs. Note that first axis parameter below
    // is the X wheelspeeds input and the second is Y wheelspeeds input.

    m_drivetrainSubsystem.setDefaultCommand(new SwerveDriveCommand(
            m_drivetrainSubsystem,
            () -> m_controller.getRightY() + m_controller.getLeftY(),
            () -> m_controller.getRightX(),
            () -> m_controller.getLeftX(),
            m_controller
    ));

    // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    //         m_drivetrainSubsystem,
    //         () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
    //         m_controller
    // ));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() 
  {
    // Back button zeros the gyroscope.
    new Button(m_controller::getXButton)
            // No requirements because we don't need to interrupt anything
            .whenPressed(m_drivetrainSubsystem::zeroGyro);

    new Button(m_controller::getYButton)
            .whenPressed(m_drivetrainSubsystem::resetModulesToForward);

    new Button(m_controller::getAButton)
            .whenPressed(m_drivetrainSubsystem::resetModulesToAbsolute);

    new Button(m_controller::getBButton)
            .whenPressed(m_drivetrainSubsystem::resetModuleEncoders);

    // Start button toggles autoRreturnToZero mode.
    new Button(m_controller::getStartButton)
            // No requirements because we don't need to interrupt anything
            .whenPressed(m_drivetrainSubsystem::toggleAutoResetToZero);

    // Back button toggles field oriented driving mode.
    new Button(m_controller::getBackButton)
            // No requirements because we don't need to interrupt anything
            .whenPressed(m_drivetrainSubsystem::toggleFieldOriented);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }
}
