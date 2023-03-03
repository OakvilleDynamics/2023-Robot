// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*We need an arm that can use an extendo-pneumatics function, and can limit how much the arm tilts/it's angle
 * There needs to be a manual and automatic stop for the arm angle limiter, and when its been met the extendo
 * arm can extend */
public class Arm extends SubsystemBase {
  // Inits motors
  private CANSparkMax bottomArm =
      new CANSparkMax(Constants.sparkArmBottomDeviceID, MotorType.kBrushed);
  private CANSparkMax topArm = new CANSparkMax(Constants.sparkArmTopDeviceID, MotorType.kBrushed);

  private RelativeEncoder bottomArmEncoder = bottomArm.getEncoder();
  private RelativeEncoder topArmEncoder = topArm.getEncoder();

  public Arm() {
    bottomArm.setInverted(Constants.bottomArmInverted);
    topArm.setInverted(Constants.topArmInverted);
  }

  /**
   * Sets the speed of the bottom arm motor to go up
   */
  public void bottomArmUp() {
    bottomArm.set(Constants.bottomArmSpeed);
  }

  /**
   * Sets the speed of the bottom arm motor to go down
   */
  public void bottomArmDown() {
    bottomArm.set(-Constants.bottomArmSpeed);
  }

  /**
   * Stops the bottom arm motor
   */
  public void bottomArmStop() {
    bottomArm.set(0);
  }

  /**
   * Sets the speed of the top arm motor to go up
   */
  public void topArmUp() {
    topArm.set(Constants.topArmSpeed);
  }

  /**
   * Sets the speed of the top arm motor to go down
   */
  public void topArmDown() {
    topArm.set(-Constants.topArmSpeed);
  }

  /**
   * Stops the top arm motor
   */
  public void topArmStop() {
    topArm.set(0);
  }

  /**
   * Gets the position of the bottom arm encoder
   * 
   * @return double position of the bottom arm encoder
   */
  public double getBottomArmEncoder() {
    return bottomArmEncoder.getPosition();
  }

  /**
   * Gets the position of the top arm encoder
   * 
   * @return double position of the top arm encoder
   */
  public double getTopArmEncoder() {
    return topArmEncoder.getPosition();
  }

  /**
   * Resets the bottom arm encoder
   * 
   * @return REVLibError error code, REVLibError.kOk if no error
   */
  public REVLibError resetBottomArmEncoder() {
    return bottomArmEncoder.setPosition(0);
  }

  /**
   * Resets the top arm encoder
   * 
   * @return REVLibError error code, REVLibError.kOk if no error
   */
  public REVLibError resetTopArmEncoder() {
    return topArmEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
