package frc.robot.swervelib.rev;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.swervelib.DriveController;
import frc.robot.swervelib.DriveControllerFactory;
import frc.robot.swervelib.ModuleConfiguration;

import static frc.robot.swervelib.rev.RevUtils.checkNeoError;

public final class NeoDriveControllerFactoryBuilder 
{
    private double nominalVoltage   = Double.NaN;
    private double currentLimit     = Double.NaN;
    private double rampRate         = Double.NaN;

    public NeoDriveControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) 
    {
        Util.consoleLog();
    
        this.nominalVoltage = nominalVoltage;

        return this;
    }

    public boolean hasVoltageCompensation() 
    {
        return Double.isFinite(nominalVoltage);
    }

    public NeoDriveControllerFactoryBuilder withCurrentLimit(double currentLimit) 
    {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() 
    {
        return Double.isFinite(currentLimit);
    }

    public NeoDriveControllerFactoryBuilder withRampRate(double rampRate) 
    {
        this.rampRate = rampRate;
        return this;
    }

    public boolean hasRampRate() 
    {
        return Double.isFinite(rampRate);
    }

    public DriveControllerFactory<ControllerImplementation, Integer> build() 
    {
        Util.consoleLog();
    
        return new FactoryImplementation();
    }

    private class FactoryImplementation implements DriveControllerFactory<ControllerImplementation, Integer> 
    {
        @Override
        public ControllerImplementation create(Integer id, ModuleConfiguration moduleConfiguration) 
        {
            Util.consoleLog();
    
            CANSparkMax motor = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);

            motor.restoreFactoryDefaults(); // 4450

            motor.setInverted(moduleConfiguration.isDriveInverted());

            if (hasVoltageCompensation())
                checkNeoError(motor.enableVoltageCompensation(nominalVoltage), "Failed to enable voltage compensation");

            if (hasCurrentLimit()) 
                checkNeoError(motor.setSmartCurrentLimit((int) currentLimit), "Failed to set current limit for NEO");
            
            if (hasRampRate())
                checkNeoError(motor.setOpenLoopRampRate(rampRate), "Failed to set NEO ramp rate");

            checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100), "Failed to set periodic status frame 0 rate");
            checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20), "Failed to set periodic status frame 1 rate");
            checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20), "Failed to set periodic status frame 2 rate");
            
            // Set neutral mode to brake
            motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

            // Setup encoder
            RelativeEncoder encoder = motor.getEncoder();
            double positionConversionFactor = Math.PI * moduleConfiguration.getWheelDiameter() * moduleConfiguration.getDriveReduction();
            encoder.setPositionConversionFactor(positionConversionFactor);
            encoder.setVelocityConversionFactor(positionConversionFactor / 60.0);

            return new ControllerImplementation(motor, encoder);
        }
    }

    private static class ControllerImplementation implements DriveController 
    {
        private final CANSparkMax       motor;
        private final RelativeEncoder   encoder;
        private double                  currentVelocity;

        private ControllerImplementation(CANSparkMax motor, RelativeEncoder encoder) 
        {
            Util.consoleLog();
    
            this.motor = motor;
            this.encoder = encoder;
        }

        @Override
        public void setReferenceVoltage(double voltage, double velocity) 
        {
            // If voltage is tiny, zero it out so no power to motor when
            // we are not actually moving. This prevents constantly applying
            // a voltage that is not enough to move the robot possibly damaging
            // the motor.
            if (Math.abs(voltage) > .25)
                motor.setVoltage(voltage);
            else
                motor.setVoltage(0);

            // We track the "requested" velocity to use as a substitute for measured
            // velocity due to problems with Neo simulation not calculating a correct
            // velocicty. TODO Fix Rev simulation to work correctly.
            currentVelocity = velocity;
        }

        public double getVoltage()
        {
            return motor.getAppliedOutput();
        }

        @Override
        public double getVelocity() 
        {
            //if (RobotBase.isReal())
                return encoder.getVelocity();
            //else
            //    return currentVelocity;
        }

        @Override
        public void stop()
        {
            motor.stopMotor();
        }

        @Override
        public RelativeEncoder getEncoder() 
        {
            return encoder;
        }

        @Override
        public CANSparkMax getMotorNeo() 
        {
            return motor;
        }

        @Override
        public TalonFX getMotor500() 
        {
            return null;
        }

        @Override
        public void setBrakeMode(boolean on) 
        {
            Util.consoleLog("%b", on);
    
            if (on)
                motor.setIdleMode(IdleMode.kBrake);
            else
                motor.setIdleMode(IdleMode.kCoast);
        }

        @Override
        public double getDistance() 
        {
            return encoder.getPosition();
        }
    }
}
