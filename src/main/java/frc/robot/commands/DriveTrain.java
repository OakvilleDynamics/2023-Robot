// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TalonDrive;

/** An example command that uses an example subsystem. */
public class DriveTrain extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TalonDrive m_subsystem;

  private final Joystick driverJoystick = new Joystick(Constants.driverJoystickID);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveTrain(TalonDrive subsystem) {
    m_subsystem = subsystem;
    System.out.println(subsystem.toString());
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);

    SmartDashboard.putData(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("TankDrive initialized");
    m_subsystem.resetEncoders();
    m_subsystem.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.drive(driverJoystick.getTwist(), driverJoystick.getY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(0.0, 0.0);
    System.out.println("TankDrive interrupted");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
