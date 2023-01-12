package frc.robot.swervelib;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

@FunctionalInterface
public interface DriveControllerFactory<Controller extends DriveController, DriveConfiguration> 
{
    default void addDashboardEntries(ShuffleboardContainer container, Controller controller) 
    {
        Util.consoleLog();
    
        container.addNumber("Current Velocity", controller::getStateVelocity);
        //TODO: remove these items when done testing.
        container.addNumber("Distance", controller::getDistance);
        container.addNumber("Voltage", controller::getVoltage);
    }

    default Controller create(
            ShuffleboardContainer container,
            DriveConfiguration driveConfiguration,
            ModuleConfiguration moduleConfiguration) 
    {
        Util.consoleLog();
    
        var controller = create(driveConfiguration, moduleConfiguration);
        
        addDashboardEntries(container, controller);

        return controller;
    }

    Controller create(DriveConfiguration driveConfiguration, ModuleConfiguration moduleConfiguration);
}
