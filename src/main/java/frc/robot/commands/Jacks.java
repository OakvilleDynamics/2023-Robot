package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.PneumaticJacks;

public class Jacks extends CommandBase {
  // subsystems
  private final PneumaticJacks m_Jacks;

  // controllers
  private final Joystick controllerJoystick = new Joystick(Constants.armJoystickID);

  public Jacks(PneumaticJacks subsystem) {
    m_Jacks = subsystem;
    addRequirements(m_Jacks);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Jacks.frontOff();
    m_Jacks.backOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (controllerJoystick.getRawButton(Constants.joystickButtonFrontJackOn) == true) {
      m_Jacks.frontOn();
      System.out.println("Jacks - front On");
    } else if (controllerJoystick.getRawButton(Constants.joystickButtonFrontJackOff) == true) {
      m_Jacks.frontOff();
      System.out.println("Jacks - front Off");
    }
    // solenoids for jacks and claw are swapped, so we changed the controls for jacks and claw (constants)

    if (controllerJoystick.getRawButton(Constants.joystickButtonBackJackOn) == true) {
      m_Jacks.backOn();
      System.out.println("Jacks - back On");
    } else if (controllerJoystick.getRawButton(Constants.joystickButtonBackJackOff) == true) {
      m_Jacks.backOff();
      System.out.println("Jacks - back Off");
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
