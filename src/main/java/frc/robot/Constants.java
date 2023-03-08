package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // PHYSICAL DEVICE IDS

  // Joystick Identifications
  public static final int driverJoystickID = 0;
  public static final int armJoystickID = 1;

  // constants for jack button assignments
  public static final int joystickButtonFrontJackOn = 7;
  public static final int joystickButtonBackJackOn = 8;
  public static final int joystickButtonFrontJackOff = 9;
  public static final int joystickButtonBackJackOff = 10;

  // Constants for claw button assignments
  public static final int joystickButtonClawOn = 1;
  public static final int joystickButtonClawOff = 2;
  public static final int joystickButtonExtendClaw = 5;
  public static final int joystickButtonRetractClaw = 6;

  // Constants for ramp button assignments
  public static final int joystickButtonRampUp = 3;
  public static final int joystickButtonRampDown = 4;

  // constants for turret control directions
  public static final int joystickTurretLeft = 3;
  public static final int joystickTurretRight = 4;

  // constants for pnuematicControllers
  public static final int pcmModuleAlpha = 25;
  public static final int pcmModuleBravo = 26;

  // constants for the pneumatic channels
  public static final int pneumaticChannelJackFrontUp = 0;
  public static final int pneumaticChannelJackFrontDown = 1;
  public static final int pneumaticChannelJackBackUp = 2;
  public static final int pneumaticChannelJackBackDown = 3;
  public static final int pneumaticChannelTransmissionHigh = 4;
  public static final int pneumaticChannelTransmissionLow = 5;
  public static final int pneumaticChannelArmRetract = 6;
  public static final int pneumaticChannelArmExtend = 7;
  public static final int pneumaticClawOpen = 0;
  public static final int pneumaticClawClosed = 1;

  // constants for motor controllers
  public static final boolean talonDriveLeftInverted = false;
  public static final boolean talonDriveRightInverted = true;
  public static final int talonDriveLeftFrontID = 13;
  public static final int talonDriveLeftMidID = 14;
  public static final int talonDriveLeftBackID = 15;
  public static final int talonDriveRightFrontID = 10;
  public static final int talonDriveRightMidID = 11;
  public static final int talonDriveRightBackID = 12;

  public static final int sparkTurretDeviceID = 16;
  public static final int sparkArmBottomDeviceID = 17;
  public static final int sparkArmTopDeviceID = 18;

  public static final int victorRampLeftDeviceID = 19;
  public static final int victorRampRightDeviceID = 20;

  // Motor feed constants for autonomous code for drive train
  public static final double motorFeedStaticGain = 0.0;
  public static final double motorFeedVelocityGain = 0.0;
  public static final double motorFeedAccelerationGain = 0.0;

  // PIDController Coefficients, left and right value are usually the same
  public static final double pidControllerProportionalCoefficient = 0.0;
  public static final double pidControllerIntegralCoefficient = 0.0;
  public static final double pidControllerDerivativeCoefficient = 0.0;

  // Distance between the center of the left and right wheels
  public static final double trackWidthMeters = 0.6223;

  // Right now the paths will use the same max velocity and acceleration
  public static final double pathDriveTrainMaxVelocity = 4;
  public static final double pathDriveTrainMaxAcceleration = 3;

  // constants for PWM motors

  // LOGICAL CONSTANTS FOR OPERATIONS

  // identifiers for gear shift
  public static final double gearThrottleHigh = -1.0;
  public static final double gearThrottleLow = 1.0;

  // constants for the bottom stage of the arm
  public static final boolean bottomArmInverted = false;
  public static final double bottomArmSpeed = 1.0;
  public static final double bottomArmP = 0.1;
  public static final double bottomArmI = 1e-4;
  public static final double bottomArmD = 1;
  public static final double bottomArmIZone = 0;
  public static final double bottomArmFF = 0;
  public static final double bottomArmMinOutput = 0;
  public static final double bottomArmMaxOutput = 0;

  // constants for the top stage of the arm
  public static final boolean topArmInverted = false;
  public static final double topArmSpeed = 1.0;
  public static final double topArmP = 0;
  public static final double topArmI = 0;
  public static final double topArmD = 0;
  public static final double topArmIZone = 0;
  public static final double topArmFF = 0;
  public static final double topArmMinOutput = 0;
  public static final double topArmMaxOutput = 0;

  // constants for the arm turret
  public static final boolean turretInverted = false;
  public static final double turretSpeed = 1.0;
  public static final double turntableP = 0;
  public static final double turntableI = 0;
  public static final double turntableD = 0;
  public static final double turntableFF = 0;
  public static final double turntableIZone = 0;
  public static final double turntableMinOutput = 0;
  public static final double turntableMaxOutput = 0;

  // constants for ramp motor rotation speed
  public static double rampSpeed = 0.5;

  public static enum ClawObjectType {
    Cone,
    Cube
  };

  public static enum ClawObjectPlacement {
    Top,
    Middle,
    Hybrid
  };

  public static enum AutoPathChoice {
    Red1,
    Red2,
    Red3,
    Blue6,
    Blue7,
    Blue8
  }

  public static final String PlaceObjectMarker = "PlaceObject";
  public static final String PickupObjectMarker = "PickUpObject";
  public static final String AutoLevelMarker = "ChargeStationLevel";
}
