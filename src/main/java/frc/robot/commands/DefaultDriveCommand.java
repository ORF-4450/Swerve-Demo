package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

import Team4450.Lib.LCD;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_throttleSupplier;
    private final DoubleSupplier m_strafeSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final XboxController m_controller;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier throttleSupplier,
                               DoubleSupplier strafeSupplier,
                               DoubleSupplier rotationSupplier,
                               XboxController controller) 
    {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_throttleSupplier = throttleSupplier;
        this.m_strafeSupplier = strafeSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.m_controller = controller;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() 
    {
        LCD.printLine(1, "x=%.3f  y=%.3f  throttle=%.3f  strafe=%.3f  rot=%.3f",
            m_controller.getLeftX(),
            m_controller.getLeftY(),
            m_throttleSupplier.getAsDouble(),
            m_strafeSupplier.getAsDouble(),
            m_rotationSupplier.getAsDouble()
        );

        LCD.printLine(2, "gyro=%.3f  yaw=%.3f",
            m_drivetrainSubsystem.getGyroRotation2d().getDegrees(),
            m_drivetrainSubsystem.getGyroYaw()
        );

        double throttle = -deadband(m_throttleSupplier.getAsDouble(), .05);
        double strafe = -deadband(m_strafeSupplier.getAsDouble(), .05);
        double rotation = -deadband(m_rotationSupplier.getAsDouble(), .05);

        // Have to invert for sim...not sure why.
        if (RobotBase.isSimulation()) rotation *= -1;

        // This seemed to really slow throttle response.
        // throttle = squareTheInput(throttle);
        // strafe = squareTheInput(strafe);
        // rotation = squareTheInput(rotation);

        m_drivetrainSubsystem.drive(throttle, strafe, rotation);

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        // m_drivetrainSubsystem.drive(
        //         ChassisSpeeds.fromFieldRelativeSpeeds(
        //                 throttle,
        //                 strafe,
        //                 rotation,
        //                 m_drivetrainSubsystem.getGyroRotation2d()
        //         )
        //);
    }

    @Override
    public void end(boolean interrupted) 
    {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
 
    private static double deadband(double value, double deadband) 
    {
        return Math.abs(value) > deadband ? value : 0.0;
    }

    private static double squareTheInput(double value) 
    {
        return Math.copySign(value * value, value);
    }
}
