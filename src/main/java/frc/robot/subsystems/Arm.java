// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*We need an arm that can use an extendo-pneumatics function, and can limit how much the arm tilts/it's angle
 * There needs to be a manual and automatic stop for the arm angle limiter, and when its been met the extendo
 * arm can extend */
public class Arm extends SubsystemBase {
  // Inits motors
  private CANSparkMax bottomArm =
      new CANSparkMax(Constants.sparkArmBottomDeviceID, MotorType.kBrushed);
  private CANSparkMax topArm = new CANSparkMax(Constants.sparkArmTopDeviceID, MotorType.kBrushed);

  private DoubleSolenoid extendArmDoubleSolenoid =
      new DoubleSolenoid(
          Constants.pcmModuleAlpha,
          PneumaticsModuleType.CTREPCM,
          Constants.pneumaticChannelArmExtend,
          Constants.pneumaticChannelArmRetract);

  public Arm() {
    bottomArm.setInverted(Constants.bottomArmInverted);
    topArm.setInverted(Constants.topArmInverted);
    retractArm();
  }

  public void bottomArmUp() {
    bottomArm.set(Constants.bottomArmSpeed);
  }

  public void bottomArmDown() {
    bottomArm.set(-Constants.bottomArmSpeed);
  }

  public void bottomArmStop() {
    bottomArm.set(0);
  }

  public void topArmUp() {
    topArm.set(Constants.topArmSpeed);
  }

  public void topArmDown() {
    topArm.set(-Constants.topArmSpeed);
  }

  public void topArmStop() {
    topArm.set(0);
  }

  public void extendArm() {
    extendArmDoubleSolenoid.set(Value.kForward);
  }

  public void retractArm() {
    extendArmDoubleSolenoid.set(Value.kReverse);
  }

  public void stopArm() {
    extendArmDoubleSolenoid.set(Value.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
