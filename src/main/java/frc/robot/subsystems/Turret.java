package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Turret extends SubsystemBase {

  // Inits motors
  private TalonSRX turntable = new TalonSRX(0);

  public Turret(){
    turntable.setInverted(true);
  }

  public void rotateLeft(double d) {
    turntable.set(ControlMode.PercentOutput, 1);
  }

  public void rotateRight() {
    turntable.set(ControlMode.PercentOutput, -1);
  }
}