// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SparkMaxDrive;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;


// Currently, the code for the arm is the same as DriveTrain.java. This is because we will be using the
// joystick to control the bottom arm, while the top arm will be using a different control scheme.
// we will probably need to change this code away from drivetrain type code once we know how the arm motors will work

public class MoveArm extends CommandBase {

/*  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SparkMaxDrive m_subsystem;
  private final Joystick armJoystick = new Joystick(Constants.controllerJoystickID);
 
  public MoveArm(SparkMaxDrive subsystem) {

    m_subsystem = subsystem;
    System.out.println(subsystem.toString());
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override

  */
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  /*  m_subsystem.drive(armJoystick.getX(), armJoystick.getY());


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