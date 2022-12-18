package frc.robot.swervelib;

import java.util.Objects;

/**
 * Additional Mk4 module configuration parameters.
 * <p>
 * The configuration parameters here are used to customize the behavior of the Mk4 swerve module.
 * Each setting is initialized to a default that should be adequate for most use cases.
 */
public class Mk4ModuleConfiguration 
{
    private double nominalVoltage       = 12.0;       // Voltage compensation value.
    private double driveCurrentLimit    = 20.0;
    private double steerCurrentLimit    = 20.0;

    private double steerP = .01, steerI = 0.0, steerD = .001; // PID factors Customized by 4450.

    public double getSteerP() { return steerP; }
    public double getSteerI() { return steerI; }
    public double getSteerD() { return steerD; }

    public double getNominalVoltage() { return nominalVoltage; }

    public void setNominalVoltage(double nominalVoltage) { this.nominalVoltage = nominalVoltage; }

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

        return Double.compare(that.getNominalVoltage(), getNominalVoltage()) == 0 && Double.compare(that.getDriveCurrentLimit(), getDriveCurrentLimit()) == 0 && Double.compare(that.getSteerCurrentLimit(), getSteerCurrentLimit()) == 0;
    }

    @Override
    public int hashCode() 
    {
        return Objects.hash(getNominalVoltage(), getDriveCurrentLimit(), getSteerCurrentLimit());
    }

    @Override
    public String toString() 
    {
        return "Mk4ModuleConfiguration{" +
                "nominalVoltage=" + nominalVoltage +
                ", driveCurrentLimit=" + driveCurrentLimit +
                ", steerCurrentLimit=" + steerCurrentLimit +
                ", p=" + steerP + ", i=" + steerI + ", d=" + steerD +
                '}';
    }
}