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

public Solenoid m_frontSolenoid = null;
public Solenoid m_backSolenoid = null;

//intialize the pneumatic brakes
  public PneumaticBrakes() {

    m_frontSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.pneumaticFrontBrakeControllerID);
    m_backSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.pneumaticBackBrakeControllerID);

  }
  //turn the brakes on 
  public void brakeOn () {

   m_frontSolenoid.set(true);
   m_backSolenoid.set(true);

  }

  // turn the brakes off
  public void brakeOff () {

    m_frontSolenoid.set(false);
    m_backSolenoid.set(false);
  }

  @Override
  public void periodic() {

  }
}