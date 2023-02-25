package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TalonDrive extends SubsystemBase {

  // Inits motors

  // The talons are not set up with the correct device numbers for now
  private WPI_TalonSRX m_leftFront = new WPI_TalonSRX(Constants.talonDriveLeftFrontID);
  private WPI_TalonSRX m_leftMid = new WPI_TalonSRX(Constants.talonDriveLeftMidID);
  private WPI_TalonSRX m_leftBack = new WPI_TalonSRX(Constants.talonDriveLeftBackID);
  private WPI_TalonSRX m_rightFront = new WPI_TalonSRX(Constants.talonDriveRightFrontID);
  private WPI_TalonSRX m_rightMid = new WPI_TalonSRX(Constants.talonDriveRightMidID);
  private WPI_TalonSRX m_rightBack = new WPI_TalonSRX(Constants.talonDriveRightBackID);

  // Inits navX
  private AHRS navxAhrs = new AHRS(SPI.Port.kMXP);

  private final DifferentialDrive m_robotDrive;

  /** Creates a new ExampleSubsystem. */
  public TalonDrive() {

    m_leftFront.setInverted(true);
    m_leftMid.setInverted(true);
    m_leftBack.setInverted(true);
    m_rightFront.setInverted(false);
    m_rightMid.setInverted(false);
    m_rightBack.setInverted(false);

    m_leftFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_rightFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    MotorControllerGroup left = new MotorControllerGroup(m_leftBack, m_leftFront, m_leftMid);
    MotorControllerGroup right = new MotorControllerGroup(m_rightBack, m_rightFront, m_rightMid);
    // add m_leftMid to left and m_rightMid to right when we change to 6 motor drivetrain
    m_robotDrive = new DifferentialDrive(left, right);
  }

  // Assigns motors

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
  /**
   * Gets the angle of the robot
   *
   * @return The current total accumulated yaw angle (Z axis) of the robot in degrees
   */
  public double getAngle() {
    return navxAhrs.getAngle();
  }

  /**
   * Gets the calibration status of the navX
   *
   * @return true if the navX is calibrating
   */
  public boolean isCalibrating() {
    return navxAhrs.isCalibrating();
  }

  /**
   * Gets the yaw of the robot
   *
   * @return The current yaw value in degrees (-180 to 180)
   */
  public double getYaw() {
    return navxAhrs.getYaw();
  }

  /**
   * Gets the pitch of the robot
   *
   * @return The current pitch value in degrees (-180 to 180)
   */
  public double getPitch() {
    return navxAhrs.getPitch();
  }

  /**
   * Gets the roll of the robot
   *
   * @return The current roll value in degrees (-180 to 180)
   */
  public double getRoll() {
    return navxAhrs.getRoll();
  }

  /**
   * Returns the "fused" (9-axis) heading.
   *
   * <p>The 9-axis heading is the fusion of the yaw angle, the tilt-corrected compass heading, and
   * magnetic disturbance detection. Note that the magnetometer calibration procedure is required in
   * order to achieve valid 9-axis headings.
   *
   * @return Fused Heading in Degrees (range 0-360)
   */
  public double getFusedHeading() {
    return navxAhrs.getFusedHeading();
  }

  /**
   * Gets the X acceleration of the robot
   *
   * @return The current acceleration in the X axis in Gs
   */
  public double getAccelX() {
    return navxAhrs.getWorldLinearAccelX();
  }

  /**
   * Gets the Y acceleration of the robot
   *
   * @return The current acceleration in the Y axis in Gs
   */
  public double getAccelY() {
    return navxAhrs.getWorldLinearAccelY();
  }

  /**
   * Gets the Z acceleration of the robot
   *
   * @return The current acceleration in the Z axis in Gs
   */
  public double getAccelZ() {
    return navxAhrs.getWorldLinearAccelZ();
  }

  /**
   * Gets the X velocity of the robot
   *
   * @return The current velocity in the X axis in m/s
   */
  public double getVelocityX() {
    return navxAhrs.getVelocityX();
  }

  /**
   * Gets the Y velocity of the robot
   *
   * @return The current velocity in the Y axis in m/s
   */
  public double getVelocityY() {
    return navxAhrs.getVelocityY();
  }

  /**
   * Gets the Z velocity of the robot
   *
   * @return The current velocity in the Z axis in m/s
   */
  public double getVelocityZ() {
    return navxAhrs.getVelocityZ();
  }

  /** Calibrates the gyro */
  public void calibrateGyro() {
    navxAhrs.calibrate();
  }

  /** Resets the gyro */
  public void resetGyro() {
    navxAhrs.reset();
  }
}
