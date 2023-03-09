package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.PneumaticClaw;

public class Claw extends CommandBase {
  // subsystems
  private final PneumaticClaw m_Claw;

  // controllers
  private final Joystick controllerJoystick = new Joystick(Constants.armJoystickID);

  public Claw(PneumaticClaw subsystem) {
    m_Claw = subsystem;
    addRequirements(m_Claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controllerJoystick.getRawButton(Constants.joystickButtonClawOff) == true) {
      m_Claw.clawOpen();
      System.out.println("Claw open.");
    } else if (controllerJoystick.getRawButton(Constants.joystickButtonClawOn) == true) {
      m_Claw.clawClose();
      System.out.println("Claw close.");
    } else {
      m_Claw.clawOff();
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Claw.clawOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
