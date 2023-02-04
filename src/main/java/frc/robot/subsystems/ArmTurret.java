package frc.robot.subsystems;

// This talon srx import is different than the TalonDrive import
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmTurret extends SubsystemBase {

  // Inits motors
  private TalonSRX turntable = new TalonSRX(15);

  public ArmTurret(){
    turntable.setInverted(true);
  }

  public void rotateLeft() {
    // We set this to -0.99 due to some bug that caused the arm to not turn when set to -1.0
    turntable.set(TalonSRXControlMode.PercentOutput, -0.99);
  }

  public void rotateRight() {
    turntable.set(TalonSRXControlMode.PercentOutput, 1.0);
  }

  public void rotateStop() {
    turntable.set(TalonSRXControlMode.PercentOutput, 0.0);
  }
}
