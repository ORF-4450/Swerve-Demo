package frc.robot.commands;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveBase;

public class ResetToForwardCommand extends CommandBase
{
    private final SwerveDriveBase   m_driveBase;

    private double                  startTime;

    public ResetToForwardCommand(SwerveDriveBase driveBase) 
    {
        Util.consoleLog();

        this.m_driveBase = driveBase;

        addRequirements(driveBase);
    }
    
    @Override
    public void initialize() 
    {
        Util.consoleLog("ResetToForwardCommand-init");
    
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
        Util.consoleLog("ResetToForwardCommand-end: interrupted=%b", interrupted);
    }
}
