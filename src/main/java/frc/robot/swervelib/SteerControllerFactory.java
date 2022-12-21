package frc.robot.swervelib;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

@FunctionalInterface
public interface SteerControllerFactory<Controller extends SteerController, SteerConfiguration> 
{
    default void addDashboardEntries(
            ShuffleboardContainer container,
            Controller controller) 
    {
        container.addNumber("2 Current Angle", () -> Math.toDegrees(controller.getStateAngle()));
        container.addNumber("3 Target Angle", () -> Math.toDegrees(controller.getReferenceAngle()));
    }

    default Controller create(
            ShuffleboardContainer dashboardContainer,
            SteerConfiguration steerConfiguration,
            ModuleConfiguration moduleConfiguration) 
    {
        var controller = create(steerConfiguration, moduleConfiguration);

        addDashboardEntries(dashboardContainer, controller);

        return controller;
    }

    Controller create(SteerConfiguration steerConfiguration, ModuleConfiguration moduleConfiguration);
}
