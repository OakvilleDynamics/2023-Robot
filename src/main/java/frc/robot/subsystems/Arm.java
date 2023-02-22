// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.components.EncodingMotor;
import frc.robot.components.SparkMax;

/*We need an arm that can use an extendo-pneumatics function, and can limit how much the arm tilts/it's angle
 * There needs to be a manual and automatic stop for the arm angle limiter, and when its been met the extendo
 * arm can extend */
public class Arm extends SubsystemBase {
  // Inits motors
  private EncodingMotor bottomArm =
      new SparkMax(Constants.sparkArmBottomDeviceID, Constants.bottomArmInverted);
  private EncodingMotor topArm =
      new SparkMax(Constants.sparkArmTopDeviceID, Constants.topArmInverted);

  public Arm() {}

  public void bottomArmUp() {
    bottomArm.setForward();
  }

  public void bottomArmDown() {
    bottomArm.setReverse();
  }

  public void bottomArmStop() {
    bottomArm.setStop();
  }

  public void topArmUp() {
    topArm.setForward();
  }

  public void topArmDown() {
    topArm.setReverse();
  }

  public void topArmStop() {
    topArm.setStop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
