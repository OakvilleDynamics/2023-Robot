package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  MotorControllerGroup m_leftMotorGroup;
  MotorControllerGroup m_rightMotorGroup;

  // Inits navX
  private AHRS navxAhrs = new AHRS(SPI.Port.kMXP);

  // Creates Odometry object to store the pose of the robot
  // Todo: fill in the values of the arguments
  private final DifferentialDriveOdometry m_odometry =
      new DifferentialDriveOdometry(
          Rotation2d.fromDegrees(getAccelX()), getLeftDistanceMeters(), getRightDistanceMeters());

  public final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(Constants.trackWidthMeters);

  private final DifferentialDrive m_robotDrive;

  /** Creates a new ExampleSubsystem. */
  public TalonDrive() {

    m_leftFront.setInverted(false);
    m_leftMid.setInverted(false);
    m_leftBack.setInverted(false);
    m_rightFront.setInverted(true);
    m_rightMid.setInverted(true);
    m_rightBack.setInverted(true);

    m_leftFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_rightFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    m_leftMotorGroup = new MotorControllerGroup(m_leftBack, m_leftFront, m_leftMid);
    m_rightMotorGroup = new MotorControllerGroup(m_rightBack, m_rightFront, m_rightMid);
    // add m_leftMid to left and m_rightMid to right when we change to 6 motor drivetrain
    m_robotDrive = new DifferentialDrive(m_leftMotorGroup, m_rightMotorGroup);

    System.out.println("Drivetrain is initialized");
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

  public void resetOdometry(Pose2d pose2D) {
    System.out.println("Reseting Odometry");
    navxAhrs.reset();
    navxAhrs.setAngleAdjustment(pose2D.getRotation().getDegrees());
    m_odometry.resetPosition(
        navxAhrs.getRotation2d().times(-1.0),
        getLeftDistanceMeters(),
        getRightDistanceMeters(),
        pose2D);
  }

  public Pose2d getPose() {
    System.out.println("Getting pose");
    return m_odometry.getPoseMeters();
  }

  private void postToSmartDashboard() {
    Pose2d pose = m_odometry.getPoseMeters();
    Translation2d position = pose.getTranslation();
    SmartDashboard.putNumber("Robot X", position.getX());
    SmartDashboard.putNumber("Robot Y", position.getY());
    SmartDashboard.putNumber("Robot Gyro", getGyro().getRadians());
  }

  /**
   * Function to retrieve latest robot gyro angle.
   *
   * @return Rotation2d object containing Gyro angle
   */
  public Rotation2d getGyro() {
    return navxAhrs.getRotation2d();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    System.out.println("Getting and setting Wheel Speeds");
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  public int getRightVelocity() {
    // Encoders are on the left and right front motor controllers
    int velocity = m_rightFront.getSensorCollection().getQuadratureVelocity();
    System.out.println("Right velocity: " + velocity);
    return velocity;
  }

  public int getLeftVelocity() {
    // Encoders are on the left and right front motor controllers
    int velocity = m_leftFront.getSensorCollection().getQuadratureVelocity();
    System.out.println("Left velocity: " + velocity);
    return velocity;
  }

  public int getRightDistanceMeters() {
    int distance = m_rightFront.getSensorCollection().getQuadraturePosition();
    System.out.println("Right distance meters: " + distance);
    return distance;
  }

  public int getLeftDistanceMeters() {
    int distance = m_leftFront.getSensorCollection().getQuadraturePosition();
    System.out.println("Left distance meters: " + distance);
    return distance;
  }

  public void outputVolts(double leftVolts, double rightVolts) {
    System.out.println("Set output volts left: " + leftVolts + " right: " + rightVolts);
    m_leftMotorGroup.setVoltage(leftVolts);
    m_rightMotorGroup.setVoltage(rightVolts);
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
    postToSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /** Resets the encoders to zero */
  public void resetEncoders() {
    m_leftFront.setSelectedSensorPosition(0);
    m_rightFront.setSelectedSensorPosition(0);
  }

  /**
   * Gets the left encoder position
   *
   * @return the left encoder position
   */
  public double getLeftEncoder() {
    return m_leftFront.getSelectedSensorPosition();
  }

  /**
   * Gets the right encoder position
   *
   * @return the right encoder position
   */
  public double getRightEncoder() {
    return m_rightFront.getSelectedSensorPosition();
  }

  /** Motor enum for getting motor controller specific values */
  public enum Motors {
    LEFT_FRONT(0),
    LEFT_MID(1),
    LEFT_BACK(2),
    RIGHT_FRONT(3),
    RIGHT_MID(4),
    RIGHT_BACK(5);

    Motors(int value) {}
  }

  /**
   * Gets the current output of the specified motor
   *
   * @param motor The motor to get the current of
   * @return The current of the specified motor in amps
   */
  public double getOutputMotorCurrent(Motors motor) {
    switch (motor) {
      case LEFT_FRONT:
        return m_leftFront.getStatorCurrent();
      case LEFT_MID:
        return m_leftMid.getStatorCurrent();
      case LEFT_BACK:
        return m_leftBack.getStatorCurrent();
      case RIGHT_FRONT:
        return m_rightFront.getStatorCurrent();
      case RIGHT_MID:
        return m_rightMid.getStatorCurrent();
      case RIGHT_BACK:
        return m_rightBack.getStatorCurrent();
      default:
        return 0;
    }
  }

  /**
   * Gets the current input of the specified motor
   *
   * @param motor The motor to get the current of
   * @return The current of the specified motor in amps
   */
  public double getInputMotorCurrent(Motors motor) {
    switch (motor) {
      case LEFT_FRONT:
        return m_leftFront.getSupplyCurrent();
      case LEFT_MID:
        return m_leftMid.getSupplyCurrent();
      case LEFT_BACK:
        return m_leftBack.getSupplyCurrent();
      case RIGHT_FRONT:
        return m_rightFront.getSupplyCurrent();
      case RIGHT_MID:
        return m_rightMid.getSupplyCurrent();
      case RIGHT_BACK:
        return m_rightBack.getSupplyCurrent();
      default:
        return 0;
    }
  }

  /**
   * Gets the bus volatge of the specified motor
   *
   * @param motor The motor to get the current of
   * @return The current of the specified motor in amps
   */
  public double getMotorBusVoltage(Motors motor) {
    switch (motor) {
      case LEFT_FRONT:
        return m_leftFront.getBusVoltage();
      case LEFT_MID:
        return m_leftMid.getBusVoltage();
      case LEFT_BACK:
        return m_leftBack.getBusVoltage();
      case RIGHT_FRONT:
        return m_rightFront.getBusVoltage();
      case RIGHT_MID:
        return m_rightMid.getBusVoltage();
      case RIGHT_BACK:
        return m_rightBack.getBusVoltage();
      default:
        return 0;
    }
  }

  /**
   * Gets the temperature of the specified motor
   *
   * @param motor The motor to get the current of
   * @return The current of the specified motor in amps
   */
  public double getMotorTemperature(Motors motor) {
    switch (motor) {
      case LEFT_FRONT:
        return m_leftFront.getTemperature();
      case LEFT_MID:
        return m_leftMid.getTemperature();
      case LEFT_BACK:
        return m_leftBack.getTemperature();
      case RIGHT_FRONT:
        return m_rightFront.getTemperature();
      case RIGHT_MID:
        return m_rightMid.getTemperature();
      case RIGHT_BACK:
        return m_rightBack.getTemperature();
      default:
        return 0;
    }
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

  /** Levels the robot out on the ground, or in this case the charge station */
  public void autoLevel() {
    double pitch = getPitch();
    double pitchRadians = Math.toRadians(pitch);

    if (pitch > 0) {
      drive(pitchRadians, 0);
    } else if (pitch < 0) {
      drive(-pitchRadians, 0);
    } else {
      drive(0, 0);
    }
  }
}
