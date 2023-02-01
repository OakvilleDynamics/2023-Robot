package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Turret extends SubsystemBase {

  // Inits motors
  private TalonSRX turntable = new TalonSRX(15);

  public Turret(){
    turntable.setInverted(true);
  }

  public void rotateLeft() {
    turntable.set(TalonSRXControlMode.PercentOutput, -0.99);
  }

  public void rotateRight() {
    turntable.set(TalonSRXControlMode.PercentOutput, 1.0);
  }

  public void rotateStop() {
    turntable.set(TalonSRXControlMode.PercentOutput, 0.0);
  }
}
