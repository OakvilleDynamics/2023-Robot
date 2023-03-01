package frc.robot.subsystems;

// This talon srx import is different than the TalonDrive import
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmTurret extends SubsystemBase {

  // Inits motors
  private CANSparkMax turntable =
      new CANSparkMax(Constants.sparkTurretDeviceID, MotorType.kBrushed);

  public ArmTurret() {
    turntable.setInverted(true);
  }

  public void rotateLeft() {
    turntable.set(Constants.turntableRotateLeft);
  }

  public void rotateRight() {
    turntable.set(Constants.turntableRotateRight);
  }

  public void rotateStop() {
    turntable.set(Constants.turntableRotateStop);
  }
}
