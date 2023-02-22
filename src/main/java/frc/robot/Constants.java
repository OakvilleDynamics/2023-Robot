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
  public static final int driverControllerPort = 0;
  public static final int armControllerPort = 1;

  // IDS TO REFER TO CAN-CONNECTED DEVICES
  // Each device has a specific device ID, visible through the Phoenix Tuner utility.
  // RB, RF, LF, LB

  public static final int[] canID = {1, 2, 4, 10};

  // Add canID of the two new motors from 6 gear motor drivetrain to the list once we change to
  // coding that robot
  // Add RM and LM as well when changing

  // PHYSICAL DEVICE IDS

  // creates the joystick identifications, 0 is for the drivetrain, 1 is for the arm and other
  // functions
  public static final int driverJoystickID = 0;
  public static final int armJoystickID = 1;
  // constants for jack button assignments
  public static final int joystickButtonFrontJackOn = 7;
  public static final int joystickButtonBackJackOn = 8;
  // Constants for claw button assignments
  public static final int joystickButtonClawOn = 1;
  // Constants for ramp button assignments
  public static final int joystickButtonRamp = 4;
  // Constants for guardrail button assignments
  public static final int joystickGuardrailOn = 6;
  public static final int joystickGuardrailOff = 7;

  // constants for pnuematicControllers
  public static final int pcmModuleAlpha = 4;
  public static final int pcmModuleBravo = 5;

  // constants for the pneumatic channels
  public static final int pneumaticGearControllerID = 1;
  public static final int pneumaticFrontJackControllerID = 3;
  public static final int pneumaticBackJackControllerID = 2;
  public static final int pneumaticClawControllerID = 4;
  public static final int pneumaticRampForwardChannel = 5;
  public static final int pneumaticRampReverseChannel = 6;
  public static final int pneumaticGuardrailControllerID = 7;

  // constants for motor controllers
  public static final int talonDriveLeftFrontID = 10;
  public static final int talonDriveLeftMidID = 11;
  public static final int talonDriveLeftBackID = 12;
  public static final int talonDriveRightFrontID = 13;
  public static final int talonDriveRightMidID = 14;
  public static final int talonDriveRightBackID = 15;

  public static final int sparkTurretDeviceID = 16;
  public static final int sparkArmBottomDeviceID = 17;
  public static final int sparkArmTopDeviceID = 18;

  // constants for PWM motors

  // LOGICAL CONSTANTS FOR OPERATIONS

  // identifiers for gear shift
  public static final double gearThrottleHigh = -1.0;
  public static final double gearThrottleLow = 1.0;

  // constants for turret control directions
  public static final int joystickTurretLeft = 3;
  public static final int joystickTurretRight = 4;
  // constants for turret rotation subsystem
  public static final double turntableRotateLeft =
      -0.99; // We set this to -0.99 due to some bug that caused the arm to not turn when set to
  // -1.0
  public static final double turntableRotateRight = 1.0;
  public static final double turntableRotateStop = 0.0;

  public static final boolean bottomArmInverted = false;

  public static final boolean topArmInverted = false;
}
