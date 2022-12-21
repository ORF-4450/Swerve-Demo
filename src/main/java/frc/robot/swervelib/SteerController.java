package frc.robot.swervelib;

import com.revrobotics.RelativeEncoder;

public interface SteerController 
{
    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    double getStateAngle();

    void stop();

    void setPidConstants(double proportional, double integral, double derivative);

    public RelativeEncoder getMotorEncoder();

    public AbsoluteEncoder getAbsoluteEncoder();

    //public double getAbsoluteOffset();

    public void setBrakeMode(boolean on);
}
