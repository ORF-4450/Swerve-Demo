package frc.robot.swervelib;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public interface DriveController 
{
    void setReferenceVoltage(double voltage);

    double getStateVelocity();

    void stop();
    
    public RelativeEncoder getEncoder();

    public CANSparkMax getMotorNeo();

    public TalonFX getMotor500();
}
