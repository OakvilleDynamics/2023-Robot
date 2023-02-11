// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.PneumaticBrakes;

public class Brakes extends CommandBase {

  //subsystems
  private final PneumaticBrakes m_Brakes;

  //controllers
  private final Joystick driverJoystick = new Joystick(Constants.driverJoystickID);
  public Brakes(PneumaticBrakes subsystem) {
        m_Brakes = subsystem;
        addRequirements(m_Brakes);

      }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (driverJoystick.getRawButton(Constants.joystickButtonBrakeOn) == true) {

      m_Brakes.brakeOn();

    }

    if (driverJoystick.getRawButton(Constants.joystickButtonBrakeOff) == true) {

      m_Brakes.brakeOff();

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
