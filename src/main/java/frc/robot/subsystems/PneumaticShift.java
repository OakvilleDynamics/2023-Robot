// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// LOOK AT
// https://docs.wpilib.org/en/stable/docs/software/hardware-apis/pneumatics/pneumatics.html#using-the-frc-control-system-to-control-pneumatics
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticShift extends SubsystemBase {

  // public DoubleSolenoid gearSolenoid = null;

  public Solenoid singleSolenoid = null;

  // intialize the pneumatic gear shifter
  public PneumaticShift() {

    singleSolenoid = new Solenoid(Constants.pcmModuleBravo, PneumaticsModuleType.CTREPCM, 7);

    /*
     * We have switched the solenoids from a double to a single, keeping until removal
     * is needed
     *
     * gearSolenoid =
     *    new DoubleSolenoid(
     *         Constants.pcmModuleBravo,
     *         PneumaticsModuleType.CTREPCM,
     *         Constants.pneumaticChannelTransmissionHigh,
     *         Constants.pneumaticChannelTransmissionLow);
     */

    SmartDashboard.putString("Gear Shift", "Low");

    System.out.println("PneumaticShift subsystem initialized");
  }

  /** Engage low gear */
  public void lowGear() {
    singleSolenoid.set(false);
    // gearSolenoid.set(Value.kForward);
    SmartDashboard.putString("Gear Shift", "Low");
  }

  /** Engage high gear */
  public void highGear() {
    singleSolenoid.set(true);
    // gearSolenoid.set(Value.kReverse);
    SmartDashboard.putString("Gear Shift", "High");
  }
}
