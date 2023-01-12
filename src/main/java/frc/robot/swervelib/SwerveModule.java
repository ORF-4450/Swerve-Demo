package frc.robot.swervelib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ModulePosition;

public interface SwerveModule 
{
    double getDriveVelocity();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    void stop();
        
    void setSteerPidConstants(double proportional, double integral, double derivative);

    void setTranslation2d(Translation2d translation);

    Translation2d getTranslation2d();
           
    double getHeadingDegrees();

    Rotation2d getHeadingRotation2d() ;

    void setModulePose(Pose2d pose) ;

    Pose2d getPose();

    void resetSteerAngleToAbsolute();

    void resetMotorEncoders();

    ModulePosition getModulePosition();

    double getAbsoluteOffset();

    void setStartingPosition();
}
