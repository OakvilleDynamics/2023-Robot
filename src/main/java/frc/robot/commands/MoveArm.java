// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {
  private final double deadZone = 0.3;

  private final Arm m_armSubsystem;

  // controllers
  private final Joystick armJoystick = new Joystick(Constants.armJoystickID);
  private final Joystick controllerJoystick = new Joystick(Constants.armJoystickID);

  public MoveArm(Arm subsystem) {
    m_armSubsystem = subsystem;
    addRequirements(m_armSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // controls bottom arm with the joystick handle
    // double armJoystickY = armJoystick.getY();
    // if (Math.abs(armJoystickY) < deadZone) {
    //   m_armSubsystem.bottomArmStop();
    // } else if (armJoystickY > 0) {
    //   m_armSubsystem.bottomArmDown();
    //   System.out.println("Bottom Arm Moving Down");
    // } else if (armJoystickY < 0) {
    //   m_armSubsystem.bottomArmUp();
    //   System.out.println("Bottom Arm Moving Up");
    // }

    // controls the arm with buttons 3 and 5
    if (armJoystick.getRawButton(2)) {
      m_armSubsystem.bottomArmUp();
    } else if (armJoystick.getRawButton(4)) {
      m_armSubsystem.bottomArmUp();
    } else {
      m_armSubsystem.stopArm();
    }

    // controls the top arm with the pov stick on the handle
    int hatPos = armJoystick.getPOV();
    if (hatPos == -1) {
      m_armSubsystem.topArmStop();
    } else if (hatPos >= 315 || hatPos <= 45) {
      m_armSubsystem.topArmUp();
      System.out.println("Top Arm Moving Up");
    } else if (hatPos >= 135 && hatPos <= 225) {
      m_armSubsystem.topArmDown();
      System.out.println("Top Arm Moving Down");
    } else {
      m_armSubsystem.topArmStop();
    }

    if (controllerJoystick.getRawButton(Constants.joystickButtonExtendClaw) == true) {
      m_armSubsystem.extendArm();
      System.out.println("Arm Extending");
    }

    if (controllerJoystick.getRawButton(Constants.joystickButtonRetractClaw) == true) {
      m_armSubsystem.retractArm();
      System.out.println("Arm Retracting");
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.bottomArmStop();
    m_armSubsystem.topArmStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
