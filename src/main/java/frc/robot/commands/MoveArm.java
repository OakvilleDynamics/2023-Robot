// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;

p/** An example command that uses an example subsystem. */
public class armCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ArmSubsystem m_subsystem;
  
    private final Joystick armJoystick =
        new Joystick(Constants.armControllerPort);
  
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public armCommand(ArmSubsystem subsystem) {
      m_subsystem = subsystem;
      System.out.println(subsystem.toString());
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(m_subsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      m_subsystem.move(armJoystick.getX(), armJoystick.getY());
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
          // m_drivetrain.drive(0.0, 0.0);
          System.out.println("Arm Move interrupted");
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }
