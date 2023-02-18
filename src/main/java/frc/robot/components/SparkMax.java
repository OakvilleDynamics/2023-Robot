package frc.robot.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

public class SparkMax implements EncodingMotor {
  private static final double forwardSpeed = 1.0;
  private static final double stopSpeed = 0.0;
  private static final double backwardSpeed = -1.0;

  private int canBusId;
  private CANSparkMax driver;
  private RelativeEncoder encoder;
  private double scalar = 1.0;

  public SparkMax(int canBusId, boolean inverted) {
    this.canBusId = canBusId;
    this.driver =
        new com.revrobotics.CANSparkMax(this.canBusId, CANSparkMaxLowLevel.MotorType.kBrushed);
    this.encoder =
        this.driver.getEncoder(com.revrobotics.SparkMaxRelativeEncoder.Type.kQuadrature, 8192);
    if (inverted) {
      this.scalar = -1.0;
    }
  }

  @Override
  public void setForward() {
    double speed = SparkMax.forwardSpeed * this.scalar;
    System.out.printf("com.revrobotics.CANSparkMax[%d].set(%f)\n", this.canBusId, speed);
    this.driver.set(speed);
  }

  @Override
  public void setReverse() {
    double speed = SparkMax.backwardSpeed * this.scalar;
    System.out.printf("com.revrobotics.CANSparkMax[%d].set(%f)\n", this.canBusId, speed);
    this.driver.set(speed);
  }

  @Override
  public void setStop() {
    double speed = SparkMax.stopSpeed * this.scalar;
    System.out.printf("com.revrobotics.CANSparkMax[%d].set(%f)\n", this.canBusId, speed);
    this.driver.set(speed);
  }

  @Override
  public double getPosition() {
    return this.encoder.getPosition();
  }

  @Override
  public double getVelocity() {
    return this.encoder.getVelocity();
  }
}
