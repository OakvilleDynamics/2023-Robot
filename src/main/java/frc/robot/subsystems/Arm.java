// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private RelativeEncoder m_bottomEncoder;
  private RelativeEncoder m_topEncoder;

  private DoubleSolenoid extendArmDoubleSolenoid =
      new DoubleSolenoid(
          Constants.pcmModuleAlpha,
          PneumaticsModuleType.CTREPCM,
          Constants.pneumaticChannelArmExtend,
          Constants.pneumaticChannelArmRetract);

  private SparkMaxPIDController bottomArmPIDController = bottomArm.getPIDController();
  private SparkMaxPIDController topArmPIDController = topArm.getPIDController();

  private static final double conversionFactor = 360;

  /** Creates a new Arm. */
  public Arm() {
    bottomArm.setInverted(Constants.bottomArmInverted);
    topArm.setInverted(Constants.topArmInverted);
    retractArm();

    System.out.println("Arm is retracted");

    m_bottomEncoder = bottomArm.getEncoder(Type.kQuadrature, 8192);
    m_topEncoder = topArm.getEncoder(Type.kQuadrature, 8192);

    bottomArmPIDController.setP(Constants.bottomArmP);
    bottomArmPIDController.setI(Constants.bottomArmI);
    bottomArmPIDController.setD(Constants.bottomArmD);
    bottomArmPIDController.setIZone(Constants.bottomArmIZone);
    bottomArmPIDController.setFF(Constants.bottomArmFF);
    bottomArmPIDController.setOutputRange(
        Constants.bottomArmMinOutput, Constants.bottomArmMaxOutput);

    topArmPIDController.setP(Constants.topArmP);
    topArmPIDController.setI(Constants.topArmI);
    topArmPIDController.setD(Constants.topArmD);
    topArmPIDController.setIZone(Constants.topArmIZone);
    topArmPIDController.setFF(Constants.topArmFF);
    topArmPIDController.setOutputRange(Constants.topArmMinOutput, Constants.topArmMaxOutput);

    bottomArmPIDController.setReference(0, ControlType.kPosition);
    topArmPIDController.setReference(0, ControlType.kPosition);

    m_topEncoder.setPositionConversionFactor(conversionFactor);
    m_bottomEncoder.setPositionConversionFactor(conversionFactor);

    System.out.println("Arm is ready");
  }

  public void bottomArmUp() {
    // Check for if the top arm has cleared the superstructure before allowing the bottom arm
    // to move
    //if (m_topEncoder.getPosition() > Constants.topArmThreshold) {
    bottomArm.set(Constants.bottomArmSpeed);
    //}
  }

  public void bottomArmDown() {
    bottomArm.set(-Constants.bottomArmSoftCloseSpeed);
  }

  public void bottomArmStop() {
    bottomArm.set(0);
  }

  public void topArmUp() {
    topArm.set(Constants.topArmSpeed);
  }

  public void topArmDown() {
    topArm.set(-Constants.topArmSoftCloseSpeed);
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
    SmartDashboard.putNumber("Top Arm Pos", m_topEncoder.getPosition());
    SmartDashboard.putNumber("Bottom Arm Pos", m_bottomEncoder.getPosition());
  }
}
