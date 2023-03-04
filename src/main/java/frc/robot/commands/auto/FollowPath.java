package frc.robot.commands.auto;

import com.pathplanner.lib.*;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.TalonDrive;

public class FollowPath extends SequentialCommandGroup {

  private final TalonDrive m_drive;

  public FollowPath(TalonDrive drivetrain, PathPlannerTrajectory traj, boolean isFirstPath) {
    System.out.println("Following Trajectory");
    m_drive = drivetrain;

    addCommands(
        new InstantCommand(
            () -> {
              // Reset odometry for the first path you run during auto
              if (isFirstPath) {
                drivetrain.resetOdometry(traj.getInitialPose());
                System.out.println("First autonomous path run.");
              } else {
                System.out.println("Not first autonomous path run.");
              }
            }),
        new PPRamseteCommand(
            traj,
            () -> drivetrain.getPose(), // Pose supplier
            new RamseteController(),
            new SimpleMotorFeedforward(
                Constants.motorFeedStaticGain,
                Constants.motorFeedVelocityGain,
                Constants.motorFeedAccelerationGain),
            new DifferentialDriveKinematics(
                Constants.trackWidthMeters), // DifferentialDriveKinematics
            () -> drivetrain.getWheelSpeeds(), // DifferentialDriveWheelSpeeds supplier
            new PIDController(
                Constants.pidControllerProportionalCoefficient,
                Constants.pidControllerIntegralCoefficient,
                Constants
                    .pidControllerDerivativeCoefficient), // Left controller. Tune these values for
            // your robot. Leaving them 0 will only use feedforwards.
            new PIDController(
                Constants.pidControllerProportionalCoefficient,
                Constants.pidControllerIntegralCoefficient,
                Constants.pidControllerDerivativeCoefficient), // Right controller (usually the same
            // values as left controller)
            (leftVolts, rightVolts) ->
                drivetrain.outputVolts(leftVolts, rightVolts), // Voltage biconsumer
            true, // Should the path be automatically mirrored depending on alliance color.
            // Optional, defaults to true
            m_drive // Requires this drive subsystem
            ));
  }
}
