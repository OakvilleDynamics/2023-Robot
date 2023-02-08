// MAKE SURE TO COMMENT/TURN INTO TEXT ON EITHER THIS OR TALONDRIVE DEPENDING ON WHAT MOTORS WE USE

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SparkMaxDrive extends SubsystemBase {

 // Inits motors
 private final CANSparkMax m_leftRear;
 private final CANSparkMax m_leftFront;
 private final CANSparkMax m_rightFront;
 private final CANSparkMax m_rightRear;
 // private final CANSparkMax m_rightMid;
 // private final CANSparkMax m_leftMid;
 // add this for 6 motor drivetrain
 
 private final DifferentialDrive m_robotDrive;


 
  /** Creates a new ExampleSubsystem. */
  public SparkMaxDrive() {
  
    

 // Assigns motors
 m_leftFront = new CANSparkMax(Constants.canID[2], MotorType.kBrushed);
 m_leftFront.setInverted(true);

 // m_leftMid = new CANSparkMax(Constants.canID[4], MotorType.kBrushed);
 // m_leftMid.setInverted(true);
 // add this code in whenever we change to the 6 motor drivetrain, change canID to whatever needed

 m_leftRear = new CANSparkMax(Constants.canID[3], MotorType.kBrushed);
 m_leftRear.setInverted(true);

 m_rightFront = new CANSparkMax(Constants.canID[1], MotorType.kBrushed);
 m_rightFront.setInverted(false);

 // m_rightMid = new CANSparkMax(Constants.canID[5], MotorType.kBrushed);
 // m_rightMid.setInverted(false);
 // add this code in whenever we change to the 6 motor drivetrain, change canID to whatever needed

 m_rightRear = new CANSparkMax(Constants.canID[0], MotorType.kBrushed);
 m_rightRear.setInverted(false);
 
 MotorControllerGroup left = new MotorControllerGroup(m_leftRear, m_leftFront);
 MotorControllerGroup right = new MotorControllerGroup(m_rightFront, m_rightRear); 
 // add m_leftMid to left and m_rightMid to right when we change to 6 motor drivetrain
 m_robotDrive = new DifferentialDrive(left, right);

  }
  
  
  /**
   * Example command factory method.
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
    if (x != 0 || y != 0) {
      System.out.println("Drive X: " + x + " Y: " + y);
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
