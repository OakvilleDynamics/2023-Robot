package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticGuardrails extends SubsystemBase {
  public Solenoid m_guardrailSolenoid = null;

  public PneumaticGuardrails() {

    m_guardrailSolenoid =
        new Solenoid(PneumaticsModuleType.CTREPCM, Constants.pneumaticGuardrailControllerID);
  }

  public void guardrailOn() {

    m_guardrailSolenoid.set(true);
  }

  public void guardrailOff() {

    m_guardrailSolenoid.set(false);
  }

  @Override
  public void periodic() {}
}
