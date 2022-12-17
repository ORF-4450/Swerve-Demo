package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

import Team4450.Lib.LCD;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final XboxController m_controller;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier,
                               XboxController controller) 
    {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.m_controller = controller;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() 
    {
        LCD.printLine(1, "x=%.3f  y=%.3f  sX=%.3f  sY=%.3f  rot=%.3f",
            m_controller.getLeftX(),
            m_controller.getLeftY(),
            m_translationXSupplier.getAsDouble(),
            m_translationYSupplier.getAsDouble(),
            m_rotationSupplier.getAsDouble()
        );

        LCD.printLine(2, "gyro=%.3f  yaw=%.3f",
            m_drivetrainSubsystem.getGyroscopeRotation().getDegrees(),
            m_drivetrainSubsystem.getGyroScopeYaw()
        );

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        m_rotationSupplier.getAsDouble(),
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
