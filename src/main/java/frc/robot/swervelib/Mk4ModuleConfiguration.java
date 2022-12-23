package frc.robot.swervelib;

import java.util.Objects;

import static frc.robot.Constants.*;

/**
 * Additional Mk4 module configuration parameters.
 * <p>
 * The configuration parameters here are used to customize the behavior of the Mk4 swerve module.
 * Each setting is initialized to a default that should be adequate for most use cases.
 */
public class Mk4ModuleConfiguration 
{
    private double nominalDriveVoltage  = DRIVE_MAX_VOLTAGE;    // Voltage compensation value. This is max voltate
                                                                // that will be output by controller.
    private double nominalSteerVoltage  = STEER_MAX_VOLTAGE;

    private double driveCurrentLimit    = DRIVE_MAX_CURRENT;
    private double steerCurrentLimit    = STEER_MAX_CURRENT;

    // Steer PID values for Neo. Customized by 4450.
    private double steerP = STEER_PID_P, steerI = STEER_PID_I, steerD = STEER_PID_D;

    public double getSteerP() { return steerP; }
    public double getSteerI() { return steerI; }
    public double getSteerD() { return steerD; }

    public double getNominalDriveVoltage() { return nominalDriveVoltage; }

    public void setNominalDriveVoltage(double nominalVoltage) { this.nominalDriveVoltage = nominalVoltage; }

    public double getNominalSteerVoltage() { return nominalSteerVoltage; }

    public void setNominalSteerVoltage(double nominalVoltage) { this.nominalSteerVoltage = nominalVoltage; }

    public double getDriveCurrentLimit() { return driveCurrentLimit; }

    public void setDriveCurrentLimit(double driveCurrentLimit) { this.driveCurrentLimit = driveCurrentLimit; }

    public double getSteerCurrentLimit() { return steerCurrentLimit; }

    public void setSteerCurrentLimit(double steerCurrentLimit) { this.steerCurrentLimit = steerCurrentLimit; }

    @Override
    public boolean equals(Object o)
    {
        if (this == o) return true;

        if (o == null || getClass() != o.getClass()) return false;

        Mk4ModuleConfiguration that = (Mk4ModuleConfiguration) o;

        return Double.compare(that.getNominalDriveVoltage(), getNominalDriveVoltage()) == 0 && 
                              Double.compare(that.getNominalSteerVoltage(), getNominalSteerVoltage()) == 0 &&
                              Double.compare(that.getDriveCurrentLimit(), getDriveCurrentLimit()) == 0 && 
                              Double.compare(that.getSteerCurrentLimit(), getSteerCurrentLimit()) == 0;
    }

    @Override
    public int hashCode() 
    {
        return Objects.hash(getNominalDriveVoltage(), getNominalSteerVoltage(), getDriveCurrentLimit(), 
                            getSteerCurrentLimit());
    }

    @Override
    public String toString() 
    {
        return "Mk4ModuleConfiguration{" +
                "nominalDriveVoltage=" + nominalDriveVoltage +
                ", nominalSteerVoltage=" + nominalSteerVoltage +
                ", driveCurrentLimit=" + driveCurrentLimit +
                ", steerCurrentLimit=" + steerCurrentLimit +
                ", p=" + steerP + ", i=" + steerI + ", d=" + steerD +
                '}';
    }
}
