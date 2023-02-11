// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// LOOK AT https://docs.wpilib.org/en/stable/docs/software/hardware-apis/pneumatics/pneumatics.html#using-the-frc-control-system-to-control-pneumatics
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;

public class PneumaticShift extends SubsystemBase {

public Solenoid gearSolenoid = null;

//intialize the pneumatic brakes
  public PneumaticShift() {

    gearSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.gearSolenoidChannel);

  }
  //turn the brakes on 
  public void lowGear () {

    gearSolenoid.set(true);

  }

  // turn the brakes off
  public void highGear () {

    gearSolenoid.set(false);

  }

  @Override
  public void periodic() {

  }
}