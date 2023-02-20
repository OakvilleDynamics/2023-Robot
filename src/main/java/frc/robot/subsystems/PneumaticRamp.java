package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticRamp extends SubsystemBase {
  public DoubleSolenoid m_rampSolenoid = null;

  public PneumaticRamp() {

    m_rampSolenoid =
        new DoubleSolenoid(
            Constants.pcmModuleBravo,
            PneumaticsModuleType.CTREPCM,
            Constants.pneumaticRampForwardChannel,
            Constants.pneumaticRampReverseChannel);
  }

  public void rampDown() {

    m_rampSolenoid.set(Value.kForward);
  }

  public void rampUp() {

    m_rampSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {}
}
