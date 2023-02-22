package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.PneumaticGuardrails;

public class MoveGuardrails extends CommandBase {
  private boolean m_isGuardrailOn = false;

  // subsystems
  private final PneumaticGuardrails m_Gaurdrail;

  // controllers
  private final Joystick driverJoystick = new Joystick(Constants.driverJoystickID);

  public MoveGuardrails(PneumaticGuardrails subsystem) {
    m_Gaurdrail = subsystem;
    addRequirements(m_Gaurdrail);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (driverJoystick.getRawButton(Constants.joystickGuardrailOn) == true) {
      if (m_isGuardrailOn) {
        m_Gaurdrail.guardrailOn();
        m_isGuardrailOn = true;
        System.out.println("Guardrail On");
      }
    }
    if (driverJoystick.getRawButton(Constants.joystickGuardrailOff) == true) {
      if (m_isGuardrailOn) {
        m_Gaurdrail.guardrailOff();
        m_isGuardrailOn = false;
        System.out.println("Guardrail Off");
      }
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
