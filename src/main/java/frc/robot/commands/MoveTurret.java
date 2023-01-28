
package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
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
        double[] inputs = procInputs(driveJoystick.getRawButtonPressed());// get if buttonTwo or buttonThree are pushed
                m_turret.rotateLeft(inputs[2]);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Processes inputs from the controller to be used on the drivetrain
     * @param xInput double X-axis input on the controller
     * @param yInput double Y-axis input on the controller
     * @param zRotate double Z-axis input on the controller
     * @param speedSlide double throttle input on the controller
     * @return double[] all values in xInput, yInput, zRotate, and speedSlide
     */


     // need to decide on what to do per button push
    private double[] procInputs(double button) {
        // if (xInput < 0.3 && xInput > -0.3)
        //     xInput = 0;

        // if (yInput < 0.3 && yInput > -0.3)
        //     yInput = 0;

        // xInput *= -1;
        // zRotate *= 0.2;
        // speedSlide = ((speedSlide * -1) + 1) / 2;
        // return new double[] { xInput, yInput, zRotate, speedSlide };
    }
}