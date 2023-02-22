// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// pneumatics enabled == slow
// pneumatics disabled == fast

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.PneumaticShift;

public class GearShift extends CommandBase {

  // subsystems
  private final PneumaticShift m_Shift;

  // controllers
  private final Joystick driverJoystick = new Joystick(Constants.driverJoystickID);

  public GearShift(PneumaticShift subsystem) {
    m_Shift = subsystem;
    addRequirements(m_Shift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (driverJoystick.getThrottle() == Constants.gearThrottleLow) {

      System.out.println("Low Gear");
      m_Shift.lowGear();
    }

    if (driverJoystick.getThrottle() == Constants.gearThrottleHigh) {

      System.out.println("High Gear");
      m_Shift.highGear();
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
