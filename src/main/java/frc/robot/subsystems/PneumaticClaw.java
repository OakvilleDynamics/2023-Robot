// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticClaw extends SubsystemBase {

  public DoubleSolenoid m_clawSolenoid = null;

  public PneumaticClaw() {

    m_clawSolenoid =
        new DoubleSolenoid(
            Constants.pcmModuleBravo,
            PneumaticsModuleType.CTREPCM,
            Constants.pneumaticClawOpen,
            Constants.pneumaticClawClosed);
  }

  public void clawOpen() {
    System.out.println("Claw open.");
    m_clawSolenoid.set(Value.kForward);
  }

  public void clawOff() {
    System.out.println("Claw off.");
    m_clawSolenoid.set(Value.kOff);
  }

  public void clawClose() {
    System.out.println("Claw close.");
    m_clawSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {}
}
