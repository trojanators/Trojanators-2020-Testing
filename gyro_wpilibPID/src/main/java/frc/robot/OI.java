/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import java.util.Map;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
public Joystick m_leftStick = new Joystick(0);
public SpeedControllerGroup  leftDriveGroup = new SpeedControllerGroup(new PWMVictorSPX(0), new PWMVictorSPX(1));
public SpeedControllerGroup rightDriveGroup = new SpeedControllerGroup(new Spark(2), new Spark (3));
public DifferentialDrive drive = new DifferentialDrive(leftDriveGroup, rightDriveGroup);
public ADXRS450_Gyro gyro = new ADXRS450_Gyro();
public NetworkTableEntry gyroAngle = Shuffleboard.getTab("Gyro").add("Gyro Angle", 0).getEntry();


PIDSource myPIDsource = new PIDSource(){

  @Override
  public void setPIDSourceType(PIDSourceType pidSource) {
    pidSource = PIDSourceType.kDisplacement;
  }

  @Override
  public double pidGet() {
    return gyro.pidGet();
  }

  @Override
  public PIDSourceType getPIDSourceType() {
    return gyro.getPIDSourceType();
  }
};

PIDOutput myPIDoutput = new PIDOutput(){

  @Override
  public void pidWrite(double output) {
    drive.arcadeDrive(0, output);
  }
};


public double throttle = 0;
public double turn = 0;

NetworkTableEntry kp = Shuffleboard.getTab("Gyro")
.add("proportional gain", 0)
.withWidget(BuiltInWidgets.kNumberSlider)
.withProperties(Map.of("min", 0, "max", 0.2))
.getEntry();

NetworkTableEntry ki = Shuffleboard.getTab("Gyro")
.add("integral gain", 0)
.withWidget(BuiltInWidgets.kNumberSlider)
.withProperties(Map.of("min", 0, "max", 0.0005))
.getEntry();

NetworkTableEntry kd = Shuffleboard.getTab("Gyro")
.add("differential gain", 0)
.withWidget(BuiltInWidgets.kNumberSlider)
.withProperties(Map.of("min", 0, "max", 0.5))
.getEntry();

NetworkTableEntry setpoint = Shuffleboard.getTab("Gyro")
.add("set point", 0)
.withWidget(BuiltInWidgets.kNumberSlider)
.withProperties(Map.of("min", 0, "max", 360))
.getEntry();

public PIDController myPID = new PIDController(kp.getDouble(0), ki.getDouble(0), kd.getDouble(0), myPIDsource, myPIDoutput);
}
