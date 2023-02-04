// MAKE SURE TO COMMENT/TURN INTO TEXT ON EITHER THIS OR TALONDRIVE DEPENDING ON WHAT MOTORS WE USE

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TalonDrive extends SubsystemBase {

 // Inits motors
 private TalonSRX m_leftFront = new TalonSRX(1);
 private TalonSRX m_leftMid = new TalonSRX(2);
 private TalonSRX m_leftBack = new TalonSRX(3);
 private TalonSRX m_rightFront = new TalonSRX(4);
 private TalonSRX m_rightMid = new TalonSRX(5);
 private TalonSRX m_rightBack = new TalonSRX(6);
 
 private final DifferentialDrive m_robotDrive;


 
  /** Creates a new ExampleSubsystem. */
  public TalonDrive() {
  
    m_leftFront.setInverted(true);
    m_leftMid.setInverted(true);
    m_leftBack.setInverted(true);
    m_rightFront.setInverted(false);
    m_rightMid.setInverted(false);
    m_rightBack.setInverted(false);

MotorControllerGroup left = new MotorControllerGroup(m_leftBack, m_leftFront, m_leftMid);
MotorControllerGroup right = new MotorControllerGroup(m_rightBack, m_rightFront, m_rightMid); 
 // add m_leftMid to left and m_rightMid to right when we change to 6 motor drivetrain
 m_robotDrive = new DifferentialDrive(left, right);

  }

  //Assigns motors
  

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
