// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// LOOK AT
// https://docs.wpilib.org/en/stable/docs/software/hardware-apis/pneumatics/pneumatics.html#using-the-frc-control-system-to-control-pneumatics
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticJacks extends SubsystemBase {

  public DoubleSolenoid m_frontSolenoid = null;
  public DoubleSolenoid m_backSolenoid = null;

  // intialize the pneumatic brakes
  public PneumaticJacks() {

    m_frontSolenoid =
        new DoubleSolenoid(
            Constants.pcmModuleAlpha,
            PneumaticsModuleType.CTREPCM,
            Constants.pneumaticChannelJackFrontDown,
            Constants.pneumaticChannelJackFrontUp);
    m_backSolenoid =
        new DoubleSolenoid(
            Constants.pcmModuleAlpha,
            PneumaticsModuleType.CTREPCM,
            Constants.pneumaticChannelJackBackDown,
            Constants.pneumaticChannelJackBackUp);

    SmartDashboard.putString("Front Jacks", "Off");
    SmartDashboard.putString("Back Jacks", "Off");

    System.out.println("PneumaticJacks subsystem initialized");
  }

  public void frontOn() {
    m_frontSolenoid.set(Value.kForward);
    SmartDashboard.putString("Front Jacks", "On");
  }

  public void frontOff() {
    m_frontSolenoid.set(Value.kReverse);
    SmartDashboard.putString("Front Jacks", "Off");
  }

  public void backOn() {
    m_backSolenoid.set(Value.kForward);
    SmartDashboard.putString("Back Jacks", "On");
  }

  public void backOff() {
    m_backSolenoid.set(Value.kReverse);
    SmartDashboard.putString("Back Jacks", "Off");
  }
}
