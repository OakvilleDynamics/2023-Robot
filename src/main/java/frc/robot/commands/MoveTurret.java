package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmTurret;

public class MoveTurret extends CommandBase {
  // Subsystems
  private final ArmTurret m_turret;

  // Controllers
  private final Joystick driveJoystick = new Joystick(Constants.armJoystickID);

  public MoveTurret(ArmTurret subsystem) {
    m_turret = subsystem;
    addRequirements(m_turret);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (driveJoystick.getRawButton(Constants.joystickTurretLeft) == true) {
      m_turret.rotateLeft();
    } else if (driveJoystick.getRawButton(Constants.joystickTurretRight) == true) {
      m_turret.rotateRight();
    } else {
      m_turret.rotateStop();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
