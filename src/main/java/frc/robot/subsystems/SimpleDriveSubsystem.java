// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SimpleDriveSubsystem extends SubsystemBase {

 // Inits motors
 private final CANSparkMax m_leftRear;
 private final CANSparkMax m_leftFront;
 private final CANSparkMax m_rightFront;
 private final CANSparkMax m_rightRear;
 
 private final DifferentialDrive m_robotDrive;


 
  /** Creates a new ExampleSubsystem. */
  public SimpleDriveSubsystem() {
  
    

 // Assigns motors
 m_leftFront = new CANSparkMax(Constants.canID[2], MotorType.kBrushed);
 m_leftFront.setInverted(true);

 m_leftRear = new CANSparkMax(Constants.canID[3], MotorType.kBrushed);
 m_leftRear.setInverted(true);

 m_rightFront = new CANSparkMax(Constants.canID[1], MotorType.kBrushed);
 m_rightFront.setInverted(false);

 m_rightRear = new CANSparkMax(Constants.canID[0], MotorType.kBrushed);
 m_rightRear.setInverted(false);
 
 MotorControllerGroup left = new MotorControllerGroup(m_leftRear, m_leftFront);
 MotorControllerGroup right = new MotorControllerGroup(m_rightFront, m_rightRear); 
 m_robotDrive = new DifferentialDrive(left, right);

  }

  //Assigns motors
  

  /**
   * Example command factory method.
   * @return a command
   */
  public void drive(double x, double y) {
    // Ignore the small movements for x and y
    if (x < 0.3 && x > -0.3) {
      x = 0;
    } 
    if (y < 0.3 && y > -0.3) {
      y = 0;
    }
    if (x != 0 || y != 0) {
      System.out.println("Drive X: " + x + " Y: " + y);
    }
    m_robotDrive.arcadeDrive(-y, -x);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
