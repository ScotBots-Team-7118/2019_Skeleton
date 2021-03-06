/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// Imports for the Robot.java class.
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Scotstants;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {
  // Object declaration.
  Gyroscope gyro;
  DriveBase driveBase;
  Autonomous auto;
  private final SendableChooser<Scotstants.Auto_Position> autoSelector = new SendableChooser<>();

  // Variable declaration.
  private Scotstants.Auto_Position auto_selection;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    // Object initialization.
    gyro = new Gyroscope();
    driveBase = new DriveBase(gyro);
    auto = new Autonomous(driveBase, gyro);

    // Adds the appropriate autonomous modes and puts them on the SmartDashboard for use.
    autoSelector.addDefault("Default", Scotstants.Auto_Position.Default);
    SmartDashboard.putData("Starting Positions", autoSelector);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * Runs once when the robot becomes disabled.
   * This is to ensure that the sensors are ready to go for the next mode
   * and that the robot doesn't do anything unexpected while disabled.
   */
  @Override
  public void disabledInit() {
    driveBase.stop();
    gyro.reset();
  }

  /**
   * Runs during autonomous mode initialization.
   */
  @Override
  public void autonomousInit() {
    auto_selection = autoSelector.getSelected();
    auto.reset();
    driveBase.enablePID();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (auto_selection) {
      case Default:
      System.out.println("ERROR: No starting position has been selected.");
      System.out.println("Autonomous action disabled. Robot action will resume during Teleop.");
      driveBase.stop();
      break;
    }
  }

  /**
   * Runs during teleop mode initialization.
   */
  @Override
  public void teleopInit() {
    driveBase.disablePID();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

  }
}
