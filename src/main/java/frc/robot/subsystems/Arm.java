// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*We need an arm that can use an extendo-pneumatics function, and can limit how much the arm tilts/it's angle
   * There needs to be a manual and automatic stop for the arm angle limiter, and when its been met the extendo
   * arm can extend */
public class Arm extends SubsystemBase {

  //Inits motors
  private PWM bottomArm = new PWM(Constants.pwmBottomArmID);
  private PWM topArm = new PWM(Constants.pwmTopArmID);
  
  public Arm() {}

  public void bottomArmUp() {
    
    bottomArm.setSpeed(Constants.bottomArmUpSpeed);

  }
  
  public void bottomArmDown() {

    bottomArm.setSpeed(Constants.bottomArmDownSpeed);

  }

  public void bottomArmStop() {

    bottomArm.setSpeed(Constants.bottomArmStopSpeed);

  }

  public void topArmUp() {
    
    topArm.setSpeed(Constants.topArmUpSpeed);

  }
  
  public void topArmDown() {

    topArm.setSpeed(Constants.topArmDownSpeed);

  }

  public void topArmStop() {

    bottomArm.setSpeed(Constants.topArmStopSpeed);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
