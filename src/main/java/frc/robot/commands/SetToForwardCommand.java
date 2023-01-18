package frc.robot.commands;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveBase;

public class SetToForwardCommand extends CommandBase
{
    private final SwerveDriveBase   m_driveBase;

    private double                  startTime;

    public SetToForwardCommand(SwerveDriveBase driveBase) 
    {
        Util.consoleLog();

        this.m_driveBase = driveBase;

        addRequirements(driveBase);
    }
    
    @Override
    public void initialize() 
    {
        Util.consoleLog();
    
        startTime = Util.timeStamp();

        m_driveBase.setModulesToForward();
    }
    
    @Override
    public boolean isFinished() 
    {
        if (Util.getElaspedTime(startTime) > 1.0) return true;

        return false;
    }

    @Override
    public void end(boolean interrupted) 
    {
        Util.consoleLog("interrupted=%b", interrupted);
    }
}
