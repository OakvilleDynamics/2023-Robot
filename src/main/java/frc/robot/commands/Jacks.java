package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.PneumaticJacks;

public class Jacks extends CommandBase {
  private boolean m_isBackJackOn = false;
  private boolean m_isFrontJackOn = false;
  // subsystems
  private final PneumaticJacks m_Jacks;

  // controllers
  private final Joystick driverJoystick = new Joystick(Constants.driverJoystickID);

  public Jacks(PneumaticJacks subsystem) {
    m_Jacks = subsystem;
    addRequirements(m_Jacks);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (driverJoystick.getRawButton(Constants.joystickButtonFrontJackOn) == true) {
      if (m_isFrontJackOn) {
        m_Jacks.frontOff();
        m_isFrontJackOn = false;
        System.out.println("frontJackOff");
      } else {
        m_Jacks.frontOn();
        m_isFrontJackOn = true;
        System.out.println("frontJackOn");
      }
    }
    if (driverJoystick.getRawButton(Constants.joystickButtonBackJackOn) == true) {
      if (m_isBackJackOn) {
        m_Jacks.backOff();
        m_isBackJackOn = false;
        System.out.println("backJackOff");
      } else {
        m_Jacks.backOn();
        m_isBackJackOn = true;
        System.out.println("backJackOn");
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
