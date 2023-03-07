// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    SmartDashboard.putString("Claw", "Off");

    System.out.println("PneumaticClaw subsystem initialized");
  }

  public void clawOpen() {
    SmartDashboard.putString("Claw", "Open");
    m_clawSolenoid.set(Value.kForward);
  }

  public void clawOff() {
    SmartDashboard.putString("Claw", "Off");
    m_clawSolenoid.set(Value.kOff);
  }

  public void clawClose() {
    SmartDashboard.putString("Claw", "Closed");
    m_clawSolenoid.set(Value.kReverse);
  }
}
