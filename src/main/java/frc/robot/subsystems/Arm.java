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


// This code is the same as SparkMaxDrive, as we are possibly using SparkMax motors for the arm
// Also, we are using a joystick to control the bottom arm, and seperate controls for the top arm
// This code will need to change to new variables and classes to work with the seperate motors used for the arm

/*We need an arm that can use an extendo-pneumatics function, and can limit how much the arm tilts/it's angle
   * There needs to be a manual and automatic stop for the arm angle limiter, and when its been met the extendo
   * arm can extend */
public class Arm extends SubsystemBase {

  /*
  // Initalize Motors
 private final CANSparkMax m_bottomArm;
 private final CANSparkMax m_topArm;


 // private final "Differential Drive" m_armControl
*/

  public Arm() {  
    
    /* 
    // Assigns Motors
    m_bottomArm = new CANSparkMax(Constants.canID[2], MotorType.kBrushed);
    m_bottomArm.setInverted(true);

    m_topArm = new CANSparkMax(Constants.canID[3], MotorType.kBrushed);
    m_topArm.setInverted(true);

    m_armControl = new "DifferentialDrive"(bottomArm, topArm);
    
    */
}

 /* 
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

 */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
