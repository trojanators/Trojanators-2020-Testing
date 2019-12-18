/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.SimPID;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI m_oi;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    m_oi.gyro.calibrate();
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
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  m_oi.gyro.reset();
  m_oi.myPID.setConstants(0.075, 0.0001, 0.4);
  m_oi.myPID.setDesiredValue(0);
  m_oi.myPID.setMaxOutput(.8);
  m_oi.myPID.setErrorEpsilon(1);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    double m_angle = m_oi.gyro.getAngle();
    m_oi.gyroAngle.setDouble(m_angle);
    //System.out.println("angle =" + m_oi.gyro.getAngle());

  //m_oi.myPID.setConstants(m_oi.kp.getDouble(0), m_oi.ki.getDouble(0), m_oi.kd.getDouble(0));
  m_oi.myPID.setDesiredValue(m_oi.setpoint.getDouble(0));
  m_oi.drive.arcadeDrive(0, m_oi.myPID.calcPID(m_angle));
}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //m_oi.myPID.setConstants(0, 0, 0);
    //m_oi.myPID.setDesiredValue(0);
    //m_oi.myPID.setMaxOutput(0.8);
    //m_oi.myPID.setErrorEpsilon(5);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  
    /* check deadband */
    if(m_oi.m_leftStick.getY() > 0.1 || m_oi.m_leftStick.getY() < -0.1){
      if(m_oi.m_leftStick.getY() < 0){
        m_oi.throttle = -Math.sqrt(Math.abs(m_oi.m_leftStick.getY()));
      }
      else{
        m_oi.throttle = Math.sqrt(m_oi.m_leftStick.getY());
      }
    }
    else{
      m_oi.throttle = 0;
    }
    /* check deadband */
    if(m_oi.m_leftStick.getRawAxis(4) > 0.1 || m_oi.m_leftStick.getRawAxis(4) < -0.1){
      if(m_oi.m_leftStick.getRawAxis(4) < 0){
        m_oi.turn = -Math.sqrt(Math.abs(m_oi.m_leftStick.getRawAxis(4)));
      }
      else{
        m_oi.turn = Math.sqrt(m_oi.m_leftStick.getRawAxis(4));
      }
    }
    else{
      m_oi.turn = 0;
    }

    m_oi.drive.arcadeDrive(-m_oi.throttle, m_oi.turn);

  }
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
