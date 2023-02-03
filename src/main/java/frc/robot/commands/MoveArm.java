// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveArm extends CommandBase {

  //subsystems 
  // public final *arm* m_arm

  //controllers
  //private final Joystick driveJoystick = new Joystick(0);


  /** Creates a new MoveArm. */
  public MoveArm() {

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Commands for moving the arm in all directions
  /*  if (driveJoystick.getRawButton(W) == true)
        {
            m_arm.moveLeft(); 
        }
    else if (driveJoystick.getRawButton(X) == true)
        {
            m_arm.moveRight();
        }
    else if (driveJoystick.getRawButton(Y) == true)
        {
            m_arm.moveUp();
        }
    else if (driveJoystick.getRawButton(Z) == true)
        {
            m_arm.moveDown();
        }    */
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
