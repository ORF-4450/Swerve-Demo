package frc.robot.swervelib;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    void stop();
        
    void setSteerPidConstants(double proportional, double integral, double derivative);
}
