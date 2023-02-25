package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.pathplanner.lib.*;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class TalonDrive extends SubsystemBase {

  // Inits motors

  private static final String kinematics = null;
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

  private void resetOdometry(Pose2d pose2D) {
    //TODO: implement
  }

  private Pose2d getPose() {
    //TODO: implement
  }

  private double getWheelSpeeds() {
    //TODO: implement
  }

  private double outputVolts() {
    //TODO: implement
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    double ksStaticGain = 0.0;
    double kvVelocityGain = 0.0;
    double kaAccelerationGain = 0.0;
    //TODO: change this code to work with differential drive, might need to make our own controller command, don't see one in WPI lib
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              this.resetOdometry(traj.getInitialPose());
          }
        }),
        new PPRamseteCommand(
            traj, 
            this::getPose, // Pose supplier
            new RamseteController(),
            new SimpleMotorFeedforward(ksStaticGain, kvVelocityGain, kaAccelerationGain),
            TalonDrive.kinematics, // DifferentialDriveKinematics
            this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
            new PIDController(0.0, 0.0, 0.0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(0.0, 0.0, 0.0), // Right controller (usually the same values as left controller)
            this::outputVolts, // Voltage biconsumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
        )
    );
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

  /**
   * Motor enum for getting motor controller specific values
   */
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
}
