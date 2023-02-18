package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticRamp extends SubsystemBase {
  public Solenoid m_rampSolenoid = null;

  public PneumaticRamp() {

    m_rampSolenoid =
        new Solenoid(PneumaticsModuleType.CTREPCM, Constants.pneumaticRampControllerID);
  }

  public void rampOn() {

    m_rampSolenoid.set(true);
  }

  public void rampOff() {

    m_rampSolenoid.set(false);
  }

  @Override
  public void periodic() {}
}
