package frc.robot.swervelib;

public interface SteerController 
{
    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    double getStateAngle();

    void stop();

    void setPidConstants(double proportional, double integral, double derivative);
}
