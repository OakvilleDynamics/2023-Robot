package frc.robot.subsystems;

// This talon srx import is different than the TalonDrive import
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmTurret extends SubsystemBase {

  // Inits motors
  private TalonSRX turntable = new TalonSRX(Constants.sparkTurretDeviceID);

  public ArmTurret() {
    turntable.setInverted(true);
  }

  public void rotateLeft() {
    turntable.set(TalonSRXControlMode.PercentOutput, Constants.turntableRotateLeft);
  }

  public void rotateRight() {
    turntable.set(TalonSRXControlMode.PercentOutput, Constants.turntableRotateRight);
  }

  public void rotateStop() {
    turntable.set(TalonSRXControlMode.PercentOutput, Constants.turntableRotateStop);
  }
}
