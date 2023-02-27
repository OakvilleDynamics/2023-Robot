// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// LOOK AT
// https://docs.wpilib.org/en/stable/docs/software/hardware-apis/pneumatics/pneumatics.html#using-the-frc-control-system-to-control-pneumatics
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticShift extends SubsystemBase {

  public DoubleSolenoid gearSolenoid = null;

  // intialize the pneumatic brakes
  public PneumaticShift() {

    gearSolenoid =
        new DoubleSolenoid(
            Constants.pcmModuleAlpha,
            PneumaticsModuleType.CTREPCM,
            Constants.pneumaticChannelTransmission,
            Constants.pneumaticChannelTransmissionBlowoff);
  }
  // turn the brakes on
  public void lowGear() {

    gearSolenoid.set(Value.kForward);
  }

  // turn the brakes off
  public void highGear() {

    gearSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {}
}
