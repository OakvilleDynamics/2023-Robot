// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RobotRamp extends SubsystemBase {
  private VictorSPX rampMotorLeft = new VictorSPX(Constants.victorRampLeftDeviceID);
  private VictorSPX rampMotorRight = new VictorSPX(Constants.victorRampRightDeviceID);

  /** Creates a new RobotRamp. */
  public RobotRamp() {
    rampMotorLeft.setInverted(false);
    rampMotorRight.setInverted(true);

    SmartDashboard.putString("Ramp Position", "Up");
  }

  /** Ramp up */
  public void rampUp() {
    rampMotorLeft.set(ControlMode.PercentOutput, Constants.rampSpeed);
    rampMotorRight.set(ControlMode.PercentOutput, Constants.rampSpeed);
    SmartDashboard.putString("Ramp Position", "Up");
  }

  /** Ramp down */
  public void rampDown() {
    rampMotorLeft.set(ControlMode.PercentOutput, -Constants.rampSpeed);
    rampMotorRight.set(ControlMode.PercentOutput, -Constants.rampSpeed);
    SmartDashboard.putString("Ramp Position", "Down");
  }

  /** Ramp stop */
  public void rampStop() {
    rampMotorLeft.set(ControlMode.PercentOutput, 0);
    rampMotorRight.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Get the bus voltage of the left ramp motor
   *
   * @return the bus voltage of the left ramp motor
   */
  public double getLeftMotorBusVoltage() {
    return rampMotorLeft.getBusVoltage();
  }

  /**
   * Get the bus voltage of the right ramp motor
   *
   * @return the bus voltage of the right ramp motor
   */
  public double getRightMotorBusVoltage() {
    return rampMotorRight.getBusVoltage();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Motor Temp", rampMotorLeft.getTemperature());
    SmartDashboard.putNumber("Right Motor Temp", rampMotorRight.getTemperature());
  }
}
