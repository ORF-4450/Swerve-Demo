package frc.robot.swervelib.rev;

import com.revrobotics.REVLibError;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.DriverStation;

public final class RevUtils 
{
    private RevUtils() {}

    public static void checkNeoError(REVLibError error, String message) 
    {
        if (error != REVLibError.kOk) 
        {
            Util.consoleLog("%s: %s", message, error.toString());

            DriverStation.reportError(String.format("%s: %s", message, error.toString()), false);
        }
    }
}
