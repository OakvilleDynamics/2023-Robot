package frc.robot.subsystems;

// This talon srx import is different than the TalonDrive import
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmTurret extends SubsystemBase {

  // Inits motors
  private CANSparkMax turntable =
      new CANSparkMax(Constants.sparkTurretDeviceID, MotorType.kBrushed);

  private RelativeEncoder m_turntableEncoder;

  private SparkMaxPIDController turntablePIDController = turntable.getPIDController();

  public ArmTurret() {
    turntable.setInverted(Constants.turretInverted);

    m_turntableEncoder = turntable.getEncoder(Type.kQuadrature, 8192);

    turntablePIDController.setP(Constants.turntableP);
    turntablePIDController.setI(Constants.turntableI);
    turntablePIDController.setD(Constants.turntableD);
    turntablePIDController.setIZone(Constants.turntableIZone);
    turntablePIDController.setFF(Constants.turntableFF);
    turntablePIDController.setOutputRange(
        Constants.turntableMinOutput, Constants.turntableMaxOutput);

    turntablePIDController.setReference(0, ControlType.kPosition);

    System.out.println("ArmTurret is ready");
  }

  public void rotateLeft() {
    turntable.set(Constants.turretSpeed);
  }

  public void rotateRight() {
    turntable.set(-Constants.turretSpeed);
  }

  public void rotateStop() {
    turntable.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Turret", m_turntableEncoder.getPosition());
  }
}
