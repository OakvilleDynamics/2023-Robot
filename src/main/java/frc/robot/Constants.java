// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

  // Add canID of the two new motors from 6 gear motor drivetrain to the list once we change to coding that robot
  // Add RM and LM as well when changing


  // PHYSICAL DEVICE IDS
  
  // creates the joystick identifications, 0 is for the drivetrain, 1 is for the arm and other functions
  public static final int driverJoystickID = 0;
  public static final int controllerJoystickID = 1;
  // constants for brake button assignments 
  public static final int joystickButtonBrakeOn = 7;
  public static final int joystickButtonBrakeOff = 8;

  // constants for pnuematicControllers
  public static final int pneumaticBrakeControllerID = 0;
  public static final int pneumaticGearControllerID = 1;

  // constants for motor controllers
  public static final int talonDriveLeftFrontID = 1;
  public static final int talonDriveLeftMidID = 2;
  public static final int talonDriveLeftBackID = 3;
  public static final int talonDriveRightFrontID = 4;
  public static final int talonDriveRightMidID = 5;
  public static final int talonDriveRightBackID = 6;
  public static final int talonTurretDeviceID = 15;



  // LOGICAL CONSTANTS FOR OPERATIONS

  // identifiers for gear shift
  public static final double gearThrottleHigh = -1.0;
  public static final double gearThrottleLow = 1.0;

  // constants for turret control directions
  public static final int joystickTurretLeft = 3;
  public static final int joystickTurretRight = 4;
  // constants for turret rotation subsystem
  public static final double turntableRotateLeft = -0.99;     // We set this to -0.99 due to some bug that caused the arm to not turn when set to -1.0
  public static final double turntableRotateRight = 1.0;
  public static final double turntableRotateStop = 0.0;


}
