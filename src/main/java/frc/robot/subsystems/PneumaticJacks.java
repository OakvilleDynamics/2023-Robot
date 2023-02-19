// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// LOOK AT
// https://docs.wpilib.org/en/stable/docs/software/hardware-apis/pneumatics/pneumatics.html#using-the-frc-control-system-to-control-pneumatics
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticJacks extends SubsystemBase {

  public Solenoid m_frontSolenoid = null;
  public Solenoid m_backSolenoid = null;

  // intialize the pneumatic brakes
  public PneumaticJacks() {

    m_frontSolenoid =
        new Solenoid(
            Constants.pcmModuleBravo,
            PneumaticsModuleType.CTREPCM,
            Constants.pneumaticFrontJackControllerID);
    m_backSolenoid =
        new Solenoid(
            Constants.pcmModuleAlpha,
            PneumaticsModuleType.CTREPCM,
            Constants.pneumaticBackJackControllerID);
  }

  public void frontOn() {
    m_frontSolenoid.set(true);
  }

  public void frontOff() {
    m_frontSolenoid.set(false);
  }

  public void backOn() {
    m_backSolenoid.set(true);
  }

  public void backOff() {
    m_backSolenoid.set(false);
  }

  @Override
  public void periodic() {}
}
