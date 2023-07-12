// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SparkClaw;

public class SparkClawCmd extends CommandBase {

  private final SparkClaw m_Claw;

  // controllers
  private final Joystick controllerJoystick = new Joystick(Constants.armJoystickID);

  public SparkClawCmd(SparkClaw subsystem) {
    m_Claw = subsystem;
    addRequirements(m_Claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controllerJoystick.getRawButton(Constants.joystickButtonReleaseClaw) == true) {
      m_Claw.releaseObject();
      System.out.println("Releasing the object from claw.");
    } else if (controllerJoystick.getRawButton(Constants.joystickButtonPickupClaw) == true) {
      m_Claw.pickupObject();
      System.out.println("Picking up the object with claw.");
    } else {
      m_Claw.stop();
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Claw.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
