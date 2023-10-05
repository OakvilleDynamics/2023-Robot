// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SparkClaw extends SubsystemBase {

  public CANSparkMax clawSparkMax =
      new CANSparkMax(Constants.sparkClawDeviceID, MotorType.kBrushless);
  /** Creates a new Claw. */
  public SparkClaw() {
    clawSparkMax.setInverted(Constants.clawInverted);
  }

  public void pickupObject() {
    clawSparkMax.set(Constants.clawIntakeSpeed);
    if (clawSparkMax.getOutputCurrent() >= 30) {
      clawSparkMax.set(Constants.clawIntakeSpeed);
    }
  }

  public void releaseObject() {
    clawSparkMax.set(Constants.clawReleaseSpeed);
  }

  public void stop() {
    clawSparkMax.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Claw Current", clawSparkMax.getOutputCurrent());
  }
}
