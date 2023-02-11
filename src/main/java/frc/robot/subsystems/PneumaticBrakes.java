// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// LOOK AT https://docs.wpilib.org/en/stable/docs/software/hardware-apis/pneumatics/pneumatics.html#using-the-frc-control-system-to-control-pneumatics
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class PneumaticBrakes extends SubsystemBase {

public Solenoid brakeSolenoid = null;

//intialize the pneumatic brakes
  public PneumaticBrakes() {

    brakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.pneumaticBrakeControllerID);

  }
  //turn the brakes on 
  public void brakeOn () {

    brakeSolenoid.set(true);

  }

  // turn the brakes off
  public void brakeOff () {

    brakeSolenoid.set(false);

  }

  @Override
  public void periodic() {

  }
}
