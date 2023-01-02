package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.*;
import frc.robot.subsystems.SwerveDriveBase;

import java.util.function.DoubleSupplier;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;

public class SwerveDriveCommand extends CommandBase 
{
    private final SwerveDriveBase m_drivetrainSubsystem;

    private final DoubleSupplier m_throttleSupplier;
    private final DoubleSupplier m_strafeSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final XboxController m_controller;
    
    private final SlewRateLimiter m_slewX = new SlewRateLimiter(THROTTLE_SLEW);
    private final SlewRateLimiter m_slewY = new SlewRateLimiter(THROTTLE_SLEW);
    private final SlewRateLimiter m_slewRot = new SlewRateLimiter(ROTATION_SLEW);

    public SwerveDriveCommand(SwerveDriveBase drivetrainSubsystem,
                               DoubleSupplier throttleSupplier,
                               DoubleSupplier strafeSupplier,
                               DoubleSupplier rotationSupplier,
                               XboxController controller) 
    {
        Util.consoleLog();

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

        double throttle = -deadband(m_throttleSupplier.getAsDouble(), THROTTLE_DEADBAND);
        double strafe = -deadband(m_strafeSupplier.getAsDouble(), THROTTLE_DEADBAND);
        double rotation = -deadband(m_rotationSupplier.getAsDouble(), ROTATION_DEADBAND);

        // Have to invert for sim...not sure why.
        if (RobotBase.isSimulation()) rotation *= -1;

        // Both squaring inputs and slew rate limiters are ways to slow down
        // or smooth response to the joystick inputs. Will test both methods.

        // Squaring seemed to really slow throttle response.
        // throttle = squareTheInput(throttle);
        // strafe = squareTheInput(strafe);
        // rotation = squareTheInput(rotation);

        //throttle = m_slewX.calculate(throttle);
        //strafe = m_slewY.calculate(strafe);
        //rotation = m_slewRot.calculate(rotation);

        m_drivetrainSubsystem.drive(throttle, strafe, rotation);
    }

    @Override
    public void end(boolean interrupted) 
    {
        Util.consoleLog("interrupted=%b", interrupted);

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
