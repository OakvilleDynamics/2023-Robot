// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.PneumaticRamp;

public class MoveRamp extends CommandBase {
  private boolean m_isRampUp = false;

  // subsystems
  private final PneumaticRamp m_Ramp;

  // controllers
  private final Joystick driverJoystick = new Joystick(Constants.driverJoystickID);

  public MoveRamp(PneumaticRamp subsystem) {
    m_Ramp = subsystem;
    addRequirements(m_Ramp);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    if (driverJoystick.getRawButton(Constants.joystickButtonRamp) == true) {
      if (m_isRampUp) {
        m_Ramp.rampDown();
        m_isRampUp = false;
        System.out.println("Ramp - down");
      } else {
        m_Ramp.rampUp();
        m_isRampUp = true;
        System.out.println("Ramp - up");
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
