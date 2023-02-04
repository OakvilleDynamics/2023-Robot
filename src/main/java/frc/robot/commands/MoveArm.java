// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class MoveArm extends CommandBase {

  //subsystems 
  // public final *arm* m_arm

  //controllers
  private final Joystick armJoystick = new Joystick(Constants.driverJoystickID);


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
  /*  if (armJoystick.getRawButton(W) == true)
        {
            m_arm.moveLeft(); 
        }
    else if (armJoystick.getRawButton(X) == true)
        {
            m_arm.moveRight();
        }
    else if (armJoystick.getRawButton(Y) == true)
        {
            m_arm.moveUp();
        }
    else if (armJoystick.getRawButton(Z) == true)
        {
            m_arm.moveDown();
        }    */

        /* We need to create a limiter for how far the arm can extend out before being able to use the extendo arm
         * To do this, we need to detect the angle of the arm somehow, and enable the extendo when the conditions are met
         * 
         * if ("arm angle limit" == true) {
         * 
         * //enable arm rotation blocker (this either will be a pneumatic blocker or a physical one)
         * m_arm.armBlocker();
         * 
         * }
         */
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
