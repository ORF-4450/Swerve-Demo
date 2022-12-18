package frc.robot.swervelib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class SwerveModuleFactory<DriveConfiguration, SteerConfiguration> 
{
    private final ModuleConfiguration moduleConfiguration;
    private final DriveControllerFactory<?, DriveConfiguration> driveControllerFactory;
    private final SteerControllerFactory<?, SteerConfiguration> steerControllerFactory;

    public SwerveModuleFactory(ModuleConfiguration moduleConfiguration,
                               DriveControllerFactory<?, DriveConfiguration> driveControllerFactory,
                               SteerControllerFactory<?, SteerConfiguration> steerControllerFactory) 
    {
        this.moduleConfiguration = moduleConfiguration;
        this.driveControllerFactory = driveControllerFactory;
        this.steerControllerFactory = steerControllerFactory;
    }

    public SwerveModule create(DriveConfiguration driveConfiguration, SteerConfiguration steerConfiguration) 
    {
        var driveController = driveControllerFactory.create(driveConfiguration, moduleConfiguration);
        var steerController = steerControllerFactory.create(steerConfiguration, moduleConfiguration);

        return new ModuleImplementation(driveController, steerController);
    }

    public SwerveModule create(ShuffleboardLayout container, DriveConfiguration driveConfiguration, SteerConfiguration steerConfiguration)
    {
        var driveController = driveControllerFactory.create(
                container,
                driveConfiguration,
                moduleConfiguration
        );

        var steerContainer = steerControllerFactory.create(
                container,
                steerConfiguration,
                moduleConfiguration
        );

        return new ModuleImplementation(driveController, steerContainer);
    }

    private static class ModuleImplementation implements SwerveModule 
    {
        private final DriveController driveController;
        private final SteerController steerController;

        private Translation2d         translation2d;
        private double                actualAngleDegrees;
        private Pose2d                pose;

        private ModuleImplementation(DriveController driveController, SteerController steerController) 
        {
            this.driveController = driveController;
            this.steerController = steerController;
        }

        @Override
        public double getDriveVelocity() 
        {
            return driveController.getStateVelocity();
        }

        @Override
        public double getSteerAngle() 
        {
            return steerController.getStateAngle();
        }

        @Override
        public void set(double driveVoltage, double steerAngle) 
        {
            steerAngle %= (2.0 * Math.PI);

            if (steerAngle < 0.0) steerAngle += 2.0 * Math.PI;

            double difference = steerAngle - getSteerAngle();

            // Change the target angle so the difference is in the range [-pi, pi) instead of [0, 2pi)

            if (difference >= Math.PI) 
                steerAngle -= 2.0 * Math.PI;
            else if (difference < -Math.PI) 
                steerAngle += 2.0 * Math.PI;
           
            difference = steerAngle - getSteerAngle(); // Recalculate difference

            // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
            // movement of the module is less than 90 deg

            if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) 
            {
                // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
                steerAngle += Math.PI;
                driveVoltage *= -1.0;
            }

            // Put the target angle back into the range [0, 2pi)

            steerAngle %= (2.0 * Math.PI);

            if (steerAngle < 0.0) steerAngle += 2.0 * Math.PI;

            driveController.setReferenceVoltage(driveVoltage);
            steerController.setReferenceAngle(steerAngle);
        }

        @Override
        public void stop()
        {
            driveController.stop();
            steerController.stop();
        }

        @Override
        public void setSteerPidConstants(double proportional, double integral, double derivative)
        {
            steerController.setPidConstants(proportional, integral, derivative);
        }

        @Override
        public void setTranslation2d(Translation2d translation) 
        {
            translation2d = translation;            
        }

        @Override
        public Translation2d getTranslation2d() 
        {
            return translation2d;
        }
        
        @Override
        public double getHeadingDegrees() 
        {
            if (RobotBase.isReal())
                return steerController.getMotorEncoder().getPosition();
            else
                return actualAngleDegrees;
        }

        @Override
        public Rotation2d getHeadingRotation2d() 
        {
            return Rotation2d.fromDegrees(getHeadingDegrees());
        }

        @Override
        public void setModulePose(Pose2d pose) 
        {
            this.pose = pose;
        }

        @Override
        public Pose2d getPose()
        {
            return pose;
        }

        @Override     
        public void resetAngleToAbsolute() 
        {
            double angle = steerController.getAbsoluteEncoder().getAbsoluteAngle() - m_turningEncoderOffset;
            steerController.getMotorEncoder().setPosition(angle);
        }

        @Override
        public void resetEncoders() 
        {
            driveController.getEncoder().setPosition(0);
            steerController.getMotorEncoder().setPosition(0);
        }
    }
} 