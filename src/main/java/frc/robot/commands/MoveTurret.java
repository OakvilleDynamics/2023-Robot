
package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class MoveTurret extends CommandBase {
    // Subsystems
    private final Turret m_turret;

    // Controllers
    private final Joystick driveJoystick = new Joystick(0);
    public MoveTurret(Turret subsystem) {
        m_turret = subsystem;
        addRequirements(m_turret);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //double[] inputs = procInputs(driveJoystick.getRawButtonPressed());// get if buttonTwo or buttonThree are pushed
        //        m_turret.rotateLeft(inputs[2]);
        if (driveJoystick.getRawButton(3) == true)
        {
            System.out.println("Button 3 Pressed");
            m_turret.rotateLeft();
        }
        else
        {
            System.out.println("Button 3 Not Pressed");
            m_turret.rotateStop();
        }

        if (driveJoystick.getRawButton(4) == true)
        {
            System.out.println("Button 4 Pressed");
            m_turret.rotateRight();
        }
        else
        {
            System.out.println("Button 4 Not Pressed");
            m_turret.rotateStop();
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}