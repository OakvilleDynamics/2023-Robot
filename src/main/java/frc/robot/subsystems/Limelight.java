// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
  // Limelight Network Table
  private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

  /** All Targeting Data from the Limelight */
  public static class TargetData {
    /**
     * Checks whether the Limelight has any valid targets (0 or 1)
     *
     * @return 0 if no valid targets, 1 if valid targets
     */
    public static double isTargetValid() {
      return limelight.getEntry("tv").getDouble(0.0);
    }

    /**
     * Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to
     * 29.8 degrees)
     *
     * @return -27 to 27 degrees
     */
    public static double getTargetXOffset() {
      return limelight.getEntry("tx").getDouble(0.0);
    }

    /**
     * Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to
     * 24.85 degrees)
     *
     * @return -20.5 to 20.5 degrees
     */
    public static double getTargetYOffset() {
      return limelight.getEntry("ty").getDouble(0.0);
    }

    /**
     * Target Area (0% of image to 100% of image)
     *
     * @return 0% of image to 100% of image
     */
    public static double getTargetArea() {
      return limelight.getEntry("ta").getDouble(0.0);
    }

    /**
     * The pipeline’s latency contribution (ms). Use getTotalLatency() for end-to-end latency.
     *
     * @return Latency in ms
     */
    public static double getTargetLatency() {
      return limelight.getEntry("tl").getDouble(0.0);
    }

    /**
     * Capture pipeline latency (ms). Time between the end of the exposure of the middle row of the
     * sensor to the beginning of the tracking pipeline.
     *
     * @return Latency in ms
     */
    public static double getPipelineLatency() {
      return limelight.getEntry("cl").getDouble(0.0);
    }

    /**
     * Total pipeline latency (ms). Sum of the capture latency and the pipeline latency.
     *
     * @return Latency in ms
     */
    public static double getTotalLatency() {
      return getPipelineLatency() + getTargetLatency();
    }

    /**
     * Sidelength of shortest side of the fitted bounding box (pixels)
     *
     * @return Pixel length
     */
    public static double getTargetShortSideLength() {
      return limelight.getEntry("tshort").getDouble(0.0);
    }

    /**
     * Sidelength of longest side of the fitted bounding box (pixels)
     *
     * @return Pixel length
     */
    public static double getTargetLongSideLength() {
      return limelight.getEntry("tlong").getDouble(0.0);
    }

    /**
     * Horizontal sidelength of the rough bounding box (0 - 320 pixels)
     *
     * @return Pixel length
     */
    public static double getTargetHorizontalSideLength() {
      return limelight.getEntry("thor").getDouble(0.0);
    }

    /**
     * Vertical sidelength of the rough bounding box (0 - 320 pixels)
     *
     * @return Pixel length
     */
    public static double getTargetVerticalSideLength() {
      return limelight.getEntry("tvert").getDouble(0.0);
    }

    /**
     * Returns the active pipeline index of the camera (0 .. 9)
     *
     * @return
     */
    public static double getPipe() {
      return limelight.getEntry("getpipe").getDouble(0.0);
    }

    /**
     * Full JSON dump of targeting results
     *
     * @return JSON String
     */
    public static String getTargetingResults() {
      return limelight.getEntry("json").getString("");
    }

    /**
     * Returns the target class (0: None, 1: Target, 2: Other)
     *
     * @return 0: None, 1: Target, 2: Other
     */
    public static double getTclass() {
      return limelight.getEntry("tclass").getDouble(0.0);
    }
  }

  /** All AprilTag/3D Data from the Limelight */
  public static class AprilTag {
    /**
     * Returns the robot's position and rotation in the field-space Translation (X,Y,Z)
     * Rotation(Roll,Pitch,Yaw) Total latency (cl+tl)
     *
     * @return Array of doubles containing the robot's position and rotation in the field-space
     *     (X,Y,Z,Roll,Pitch,Yaw) and total latency (cl+tl)
     */
    public static double[] getBotPose() {
      return limelight.getEntry("botpose").getDoubleArray(new double[6]);
    }

    /**
     * Returns the robot's position and rotation in the field-space for the blue alliance
     * Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw) Total latency (cl+tl)
     *
     * @return Array of doubles containing the robot's position and rotation in the field-space
     *     (X,Y,Z,Roll,Pitch,Yaw) and total latency (cl+tl)
     */
    public static double[] getBotPose_BlueAlliance() {
      return limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    }

    // Robot transform in field-space. Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency
    // (cl+tl)
    /**
     * Returns the robot's position and rotation in the field-space for the red alliance Translation
     * (X,Y,Z) Rotation(Roll,Pitch,Yaw) Total latency (cl+tl)
     *
     * @return Array of doubles containing the robot's position and rotation in the field-space
     *     (X,Y,Z,Roll,Pitch,Yaw) and total latency (cl+tl)
     */
    public static double[] getBotPose_RedAlliance() {
      return limelight.getEntry("botpose_wpired").getDoubleArray(new double[6]);
    }

    /**
     * 3D transform of the camera in the coordinate system of the primary in-view AprilTag (array
     * (6))
     *
     * @return Array of doubles containing the camera's position and rotation in the target-space of
     *     the primary in-view AprilTag (X,Y,Z,Roll,Pitch,Yaw)
     */
    public static double[] getCameraPoseTargetSpace() {
      return limelight.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
    }

    /**
     * 3D transform of the primary in-view AprilTag in the coordinate system of the Camera (array
     * (6))
     *
     * @return Array of doubles containing the primary in-view AprilTag's position and rotation in
     *     the camera-space (X,Y,Z,Roll,Pitch,Yaw)
     */
    public static double[] getTargetPoseCameraSpace() {
      return limelight.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
    }

    /**
     * 3D transform of the primary in-view AprilTag in the coordinate system of the Robot (array
     * (6))
     *
     * @return Array of doubles containing the primary in-view AprilTag's position and rotation in
     *     the robot-space (X,Y,Z,Roll,Pitch,Yaw)
     */
    public static double[] getTargetPoseRobotSpace() {
      return limelight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    }

    /**
     * 3D transform of the robot in the coordinate system of the primary in-view AprilTag (array
     * (6))
     *
     * @return Array of doubles containing the robot's position and rotation in the target-space of
     *     the primary in-view AprilTag (X,Y,Z,Roll,Pitch,Yaw)
     */
    public static double[] getBotPoseTargetSpace() {
      return limelight.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    }

    /**
     * 3D transform of the camera in the coordinate system of the robot (array (6))
     *
     * @return Array of doubles containing the camera's position and rotation in the robot-space
     *     (X,Y,Z,Roll,Pitch,Yaw)
     */
    public static double[] getCameraPoseRobotSpace() {
      return limelight.getEntry("camerapose_robotspace").getDoubleArray(new double[6]);
    }

    /**
     * Returns the primary in-view AprilTag's ID
     *
     * @return ID of the primary in-view AprilTag
     */
    public static double[] getPrimaryAprilTag() {
      return limelight.getEntry("tid").getDoubleArray(new double[6]);
    }
  }

  /** All Camera Controls from the Limelight */
  public static class CameraControls {
    /**
     * Sets the LED mode of the Limelight0 = use the LED Mode set in the current pipeline 1 = force
     * off 2 = force blink 3 = force on
     *
     * @param mode LED Mode integer
     */
    public static void setLEDMode(int mode) {
      limelight.getEntry("ledMode").setNumber(mode);
    }

    /**
     * Sets the camera mode of the Limelight 0 = Vision Processor 1 = Driver Camera (Increases
     * exposure, disables vision processing)
     *
     * @param mode Camera Mode integer
     */
    public static void setCameraMode(int mode) {
      limelight.getEntry("camMode").setNumber(mode);
    }

    /**
     * Sets the pipeline of the Limelight Has to be within the range of 0-9
     *
     * @param pipeline Pipeline integer
     */
    public static void setPipeline(int pipeline) {
      limelight.getEntry("pipeline").setNumber(pipeline);
    }

    /**
     * Sets the stream mode of the Limelight 0 = Standard - Side-by-side streams if a webcam is
     * attached to Limelight 1 = PiP Main - The secondary camera stream is placed in the lower-right
     * corner of the primary camera stream 2 = PiP Secondary - The primary camera stream is placed
     * in the lower-right corner of the secondary camera stream
     *
     * @param mode Stream Mode integer
     */
    public static void setStreamMode(int mode) {
      limelight.getEntry("stream").setNumber(mode);
    }

    /**
     * Sets the snapshot mode of the Limelight 0 = Stop taking snapshots 1 = Take two snapshots per
     * second
     *
     * @param mode Snapshot Mode integer
     */
    public static void setSnapshotMode(int mode) {
      limelight.getEntry("snapshot").setNumber(mode);
    }

    /**
     * Sets the crop of rectangle. The pipeline must ustilize the default crop rectangle in the web
     * interface The array must have exactly 4 elements
     *
     * @param crop Array of doubles containing the crop rectangle
     */
    public static void setCrop(double[] crop) {
      limelight.getEntry("crop").setDoubleArray(crop);
    }
  }
}
