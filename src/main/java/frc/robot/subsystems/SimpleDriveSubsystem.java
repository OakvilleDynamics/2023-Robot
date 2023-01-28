// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimpleDriveSubsystem extends SubsystemBase {
  private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  
  /** Creates a new ExampleSubsystem. */
  public SimpleDriveSubsystem() {}

  //Assigns motors
  

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void drive(double x, double y) {
    // Ignore the small movements for x and y
    if (x < 0.3 && x > -0.3) {
      x = 0;
    } 
    if (y < 0.3 && y > -0.3) {
      y = 0;
    }
    m_robotDrive.arcadeDrive(-y, -x);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
