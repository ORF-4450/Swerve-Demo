// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.unmanaged.Unmanaged;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.REVPhysicsSim;

import frc.robot.swervelib.Mk4iSwerveModuleHelper;
import frc.robot.swervelib.SdsModuleConfigurations;
import frc.robot.swervelib.SwerveModule;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;

import static frc.robot.Constants.*;

public class SwerveDriveBase extends SubsystemBase 
{
  private boolean       autoReturnToZero = false, fieldOriented = true, overrideAutoResetToZero;

  private SimDouble     simAngle; // navx sim.

  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 4.0; //12.0;

  //  Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * 
  //                   SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI

  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 5676.0 / 60.0 *
          SdsModuleConfigurations.MK4I_L1.getDriveReduction() *
          SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;

  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */

  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
          
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
      getGyroRotation2d(),
      new Pose2d(),
      m_kinematics,
      VecBuilder.fill(0.1, 0.1, 0.1),
      VecBuilder.fill(0.05),
      VecBuilder.fill(0.1, 0.1, 0.1));

  private final Field2d     field2d = new Field2d();

  public SwerveDriveBase() 
  {
    Util.consoleLog();

    // This thread will wait a bit and then reset the gyro while this constructor
    // continues to run. We do this because we have to wait a bit to reset the
    // gyro after creating it.

    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroGyro();
      } catch (Exception e) { }
    }).start();

    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    if (RobotBase.isSimulation()) 
    {
      var dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");

      simAngle = new SimDouble((SimDeviceDataJNI.getSimValueHandle(dev, "Yaw")));
    }

    SmartDashboard.putData("Field2d", field2d);

    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk4iSwerveModuleHelper.createFalcon500(...)
    //   Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk4iSwerveModuleHelper.createNeo(...)
    //   Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk4iSwerveModuleHelper.createFalcon500Neo(...)
    //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
    //
    // Mk49SwerveModuleHelper.createNeoFalcon500(...)
    //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

    // By default the swerve modules are created with the default Mk4ModuleConfiguration object.
    // The Mk4ModuleConfiguration object contains all of the configurable tuning parameters available for
    // modules. If you wish to adjust this configuration, create a Mk4MmoduleConfiguration object here and
    // call it's methods to set the parameters you wish to adjust and then pass that configuration object 
    // to each of the createNeo calls below, adding the configuration object just ahead of the GearRatio
    // parameter. The Mk4ModuleConfiguration is currently customized for Neos.
    
    m_frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
            ModulePosition.FL,
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk4iSwerveModuleHelper.GearRatio.L1,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // Sets the module center translation from center of robot.
    m_frontLeftModule.setTranslation2d(new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0));
    
    // We will do the same for the other modules

    m_frontRightModule = Mk4iSwerveModuleHelper.createNeo(
            ModulePosition.FR,
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_frontRightModule.setTranslation2d(new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));
    
    m_backLeftModule = Mk4iSwerveModuleHelper.createNeo(
            ModulePosition.BL,
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backLeftModule.setTranslation2d(new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0));

    m_backRightModule = Mk4iSwerveModuleHelper.createNeo(
            ModulePosition.BR,
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );

    m_backRightModule.setTranslation2d(new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));
    
    resetModuleEncoders();
    //resetModulesToAbsolute();

    // Set starting position on field.
    setOdometry(new Pose2d(1.03, 2.825, new Rotation2d(0)));
    
    updateDS();
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyro() 
  {
    Util.consoleLog();

    m_navx.zeroYaw();
  }

  public Rotation2d getGyroRotation2d() 
  {
    if (m_navx.isMagnetometerCalibrated()) 
    {
     // We will only get valid fused headings if the magnetometer is calibrated
     return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    }

    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    //return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
    return Rotation2d.fromDegrees(-m_navx.getYaw());
  }

  public double getGyroYaw()
  {
    return -m_navx.getYaw();
  }

  public double getHeadingDegrees() 
  {
    return Math.IEEEremainder((-m_navx.getAngle()), 360);
  }

  public Rotation2d getHeadingRotation2d() 
  {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  /**
   * Set the chassis speeds object that the periodic function executes.
   * @param chassisSpeeds ChassisSpeeds object to execute.
   */
  public void drive(ChassisSpeeds chassisSpeeds) 
  {
    m_chassisSpeeds = chassisSpeeds;
  }

  /**
   * Generate a chassis speeds object that the periodic funtion executes from the
   * directional inputs. Speeds in m/s. Optionally turns on the auto return to zero
   * position function for one periodic call. Used to align wheels to zero even when 
   * auto return to zero is off.
   * @param throttle Throttle speed.
   * @param strafe Strafe speed.
   * @param rotation Rotation speed.
   * @param override True to override auto return to zero function.
   */
  public void drive(double throttle, double strafe, double rotation, boolean override)
  {
      overrideAutoResetToZero = override;

      if (overrideAutoResetToZero) autoReturnToZero = true;

      drive(throttle, strafe, rotation);
  }

  /**
   * Generate a chassis speeds object that the periodic funtion executes from the
   * directional inputs. Speeds in m/s.
   * @param throttle Throttle speed.
   * @param strafe Strafe speed.
   * @param rotation Rotation speed.
   */
  public void drive(double throttle, double strafe, double rotation)
  {
    // Convert joystick values into speeds.

    throttle *= MAX_VELOCITY_METERS_PER_SECOND;
    strafe *= MAX_VELOCITY_METERS_PER_SECOND;
    rotation *= MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    // Create chassis speeds in either field or robot drive orientation.

    m_chassisSpeeds = fieldOriented
        ? ChassisSpeeds.fromFieldRelativeSpeeds(throttle, strafe, rotation, getHeadingRotation2d())
        : new ChassisSpeeds(throttle, strafe, rotation);
  }

  /**
   * Drives the robot with the currently set chassis speeds object on each scheduler pass.
   */
  @Override
  public void periodic() 
  {
    LCD.printLine(3, "max vel=%.3fms  max ang vel=%.3frs  voltage=%.1f",
        MAX_VELOCITY_METERS_PER_SECOND,
        MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        MAX_VOLTAGE
    );

    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    if (!autoReturnToZero && states[0].speedMetersPerSecond < 0.01)
        m_frontLeftModule.stop();
    else
        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        
    if (!autoReturnToZero && states[1].speedMetersPerSecond < 0.01)
        m_frontRightModule.stop();
    else
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
  
    if (!autoReturnToZero && states[2].speedMetersPerSecond < 0.01)
        m_backLeftModule.stop();
    else
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());

    if (!autoReturnToZero && states[3].speedMetersPerSecond < 0.01)
        m_backRightModule.stop();
    else
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());

    if (overrideAutoResetToZero) autoReturnToZero = false;

    overrideAutoResetToZero = false;

    updateOdometry(states);

    field2d.setRobotPose(getPoseMeters());

    setField2dModulePoses();
  }

  /**
   * Update robot pose (position & rotation) on the field. Used to drive
   * the field2d object.
   * @param states Array of module state objects.
   */
  public void updateOdometry(SwerveModuleState[] states)
  {
    m_odometry.update(getHeadingRotation2d(), states);

    updateModulePose(m_frontLeftModule);
    updateModulePose(m_frontRightModule);
    updateModulePose(m_backLeftModule);
    updateModulePose(m_backRightModule);
  }

  /**
   * Update the pose of a swerve module on the field2d object. Module
   * pose is connected to the robot pose so they move together on the
   * field sim but this code will rotate the module icon to direction wheel
   * is pointing.
   * @param module Swerve module to update.
   */
  private void updateModulePose(SwerveModule module)
  {
    Translation2d modulePosition = module.getTranslation2d()
        //.rotateBy(getHeadingRotation2d())
        .rotateBy(getPoseMeters().getRotation())
        .plus(getPoseMeters().getTranslation());
    
    module.setModulePose(
        new Pose2d(modulePosition, module.getHeadingRotation2d().plus(getHeadingRotation2d())));
  }

  private void setField2dModulePoses()
  {
    Pose2d      modulePoses[] = new Pose2d[4];
    
    modulePoses[0] = m_frontLeftModule.getPose();
    modulePoses[1] = m_frontRightModule.getPose();
    modulePoses[2] = m_backLeftModule.getPose();
    modulePoses[3] = m_backRightModule.getPose();

    field2d.getObject("Swerve Modules").setPoses(modulePoses);
  }

  public Pose2d getPoseMeters() 
  {
    return m_odometry.getEstimatedPosition();
  }

  public SwerveDrivePoseEstimator getOdometry() 
  {
    return m_odometry;
  }

  public void setOdometry(Pose2d pose) 
  {
    m_odometry.resetPosition(pose, pose.getRotation());
    m_navx.reset();
  }  

  @Override
  public void simulationPeriodic() 
  {
    // Assumes Neos. SIM for 500s not implemented.
    //REVPhysicsSim.getInstance().run();
 
    // want to simulate navX gyro changing as robot turns
    // information available is radians per second and this happens every 20ms
    // radians/2pi = 360 degrees so 1 degree per second is radians / 2pi
    // increment is made every 20 ms so radian adder would be (rads/sec) *(20/1000)
    // degree adder would be radian adder * 360/2pi
    // so degree increment multiplier is 360/100pi = 1.1459

    double temp = m_chassisSpeeds.omegaRadiansPerSecond * 1.1459155;

    temp += simAngle.get();

    simAngle.set(temp);

    Unmanaged.feedEnable(20);
  }

  public void toggleAutoResetToZero()
  {
    Util.consoleLog();
    
     autoReturnToZero = !autoReturnToZero;
     updateDS();
  }

  public boolean getAutoResetToZero()
  {
     return autoReturnToZero;
  }

  public void toggleFieldOriented()
  {
      Util.consoleLog();
    
      fieldOriented = !fieldOriented;
      updateDS();
  }

  public boolean getFieldOriented()
  {
      return fieldOriented;
  }

  public void updateDS()
  {
      SmartDashboard.putBoolean("Field Oriented", fieldOriented);
      SmartDashboard.putBoolean("Auto Return To Zero", autoReturnToZero);
  }

  public void resetModuleEncoders() 
  {
      Util.consoleLog();
    
      m_frontLeftModule.resetMotorEncoders(); 
      m_frontRightModule.resetMotorEncoders(); 
      m_backLeftModule.resetMotorEncoders(); 
      m_backRightModule.resetMotorEncoders(); 
  }
  
  public void resetModulesToAbsolute() 
  {
      m_frontLeftModule.resetSteerAngleToAbsolute();
      m_frontRightModule.resetSteerAngleToAbsolute();
      m_backLeftModule.resetSteerAngleToAbsolute();
      m_backRightModule.resetSteerAngleToAbsolute();
  }

  public void resetModulesToForward()
  {
      Util.consoleLog("reset to forward");
      
      drive(0, 0, 0, true);
  }
}
