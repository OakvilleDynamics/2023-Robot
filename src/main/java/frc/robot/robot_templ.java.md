// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/**
 * This is a demo program showing the use of thfe DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final Joystick m_stick = new Joystick(0);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);
  }

  /* (non-Javadoc)
   * @see edu.wpi.first.wpilibj.IterativeRobotBase#teleopPeriodic()
   */
  @Override
  public void teleopPeriodic(){
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    double speedSlider = (m_stick.getThrottle() + 1) / 2;

    m_robotDrive.arcadeDrive(-m_stick.getY() * speedSlider, -m_stick.getX() * speedSlider);

    System.out.println("SPEED SLIDER: " + speedSlider);
    System.out.println("X-AXIS: " + -m_stick.getX() * speedSlider);
    System.out.println("Y-AXIS: " + -m_stick.getY() * speedSlider);
    System.out.println("Z-AXIS:"+ -m_stick.getZ() * speedSlider);
   m_robotDrive.tankDrive() stopMotor();// stop robot
   m_leftMotor.stopMotor();//stop robot
   m_leftMotor.set(speedSlider);
   m_rightMotor.set(speedSlider);//start robot
   
  
   start.motorcontrol;//stop robot start robot) m_leftMotor m_rightMotor.set;(speedSlider);//stop robot start robot

  }
   