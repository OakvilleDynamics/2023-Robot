// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {

  private final Arm m_armSubsystem;

  // controllers
  private final Joystick controllerJoystick = new Joystick(Constants.controllerJoystickID);

  public MoveArm(Arm subsystem) {
    m_armSubsystem = subsystem;
    addRequirements(m_armSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    // controls bottom arm with the joystick handle
    if (controllerJoystick.getY() <= 0.3) {
      m_armSubsystem.bottomArmDown();
    } else if (controllerJoystick.getY() >= -0.3) {
      m_armSubsystem.bottomArmUp();
    } else {
      m_armSubsystem.bottomArmStop();
    }

    // controls the top arm with the pov stick on the handle
    if (controllerJoystick.getPOV() == 0) {
      m_armSubsystem.topArmUp();
    } else if (controllerJoystick.getPOV() == 180) {
      m_armSubsystem.topArmDown();
    } else {
      m_armSubsystem.topArmStop();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
