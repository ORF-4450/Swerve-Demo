package frc.robot.swervelib;

public interface DriveController 
{
    void setReferenceVoltage(double voltage);

    double getStateVelocity();

    void stop();
}
