package frc.robot.subsystems;

// This talon srx import is different than the TalonDrive import
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmTurret extends SubsystemBase {

  // Inits motors
  private CANSparkMax turntable =
      new CANSparkMax(Constants.sparkTurretDeviceID, MotorType.kBrushed);

  private RelativeEncoder m_turntableEncoder;

  public ArmTurret() {
    turntable.setInverted(false);

    m_turntableEncoder = turntable.getEncoder(Type.kQuadrature, 8192);

    System.out.println("ArmTurret is ready");
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Turret", m_turntableEncoder.getPosition());
  }
}
