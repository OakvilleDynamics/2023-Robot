// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticClaw extends SubsystemBase {

  public Solenoid m_clawSolenoid = null;

  public PneumaticClaw() {

    m_clawSolenoid =
        new Solenoid(
            Constants.pcmModuleAlpha,
            PneumaticsModuleType.CTREPCM,
            Constants.pneumaticClawControllerID);
  }

  public void clawOn() {

    m_clawSolenoid.set(true);
  }

  public void clawOff() {

    m_clawSolenoid.set(false);
  }

  @Override
  public void periodic() {}
}
