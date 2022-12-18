package frc.robot.swervelib;

import com.revrobotics.RelativeEncoder;

public interface DriveController 
{
    void setReferenceVoltage(double voltage);

    double getStateVelocity();

    void stop();
    
    public RelativeEncoder getEncoder();
}
