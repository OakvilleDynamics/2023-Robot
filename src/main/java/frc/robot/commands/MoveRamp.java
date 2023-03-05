// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.RobotRamp;

public class MoveRamp extends CommandBase {

  // subsystems
  private final RobotRamp m_Ramp;

  // controllers
  private final Joystick driverJoystick = new Joystick(Constants.driverJoystickID);

  public MoveRamp(RobotRamp subsystem) {
    m_Ramp = subsystem;
    addRequirements(m_Ramp);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    if (driverJoystick.getRawButton(Constants.joystickButtonRampUp) == true) {
      m_Ramp.rampUp();
      System.out.println("Ramp - up");
    }

    if (driverJoystick.getRawButton(Constants.joystickButtonRampDown) == true) {
      m_Ramp.rampDown();
      System.out.println("Ramp - down");
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
