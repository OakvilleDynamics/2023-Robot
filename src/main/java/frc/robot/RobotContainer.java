// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.*;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoPathChoice;
import frc.robot.commands.*;
import frc.robot.commands.auto.*;
import frc.robot.components.AutoPath;
import frc.robot.subsystems.*;

/* This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static final RobotContainer m_robotContainer = new RobotContainer();

  // The robot's subsystems
  public final Arm m_arm = new Arm();
  public final ArmTurret m_turret = new ArmTurret();
  private final TalonDrive m_simpledrive = new TalonDrive();
  public final PneumaticJacks m_Jacks = new PneumaticJacks();
  public final PneumaticShift m_Shift = new PneumaticShift();
  public final RobotRamp m_ramp = new RobotRamp();
  public final PneumaticClaw m_claw = new PneumaticClaw();

  private int m_autonomousRuns = 0;

  SendableChooser<AutoPath> chooser = new SendableChooser<>();

  // Autonomous Commands
  // private final Command m_placeobject = new PlaceObject(m_turret);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_arm.setDefaultCommand(new MoveArm(m_arm));
    m_turret.setDefaultCommand(new MoveTurret(m_turret));
    m_simpledrive.setDefaultCommand(new DriveTrain(m_simpledrive));
    m_Jacks.setDefaultCommand(new Jacks(m_Jacks));
    m_Shift.setDefaultCommand(new GearShift(m_Shift));
    m_ramp.setDefaultCommand(new MoveRamp(m_ramp));

    chooser.setDefaultOption("Do nothing", null);
    chooser.addOption(
        "Red AprilTag 1", new AutoPath(AutoPathChoice.Red1, m_simpledrive, m_arm, m_claw));
    chooser.addOption(
        "Red AprilTag 2", new AutoPath(AutoPathChoice.Red2, m_simpledrive, m_arm, m_claw));
    chooser.addOption(
        "Red AprilTag 3", new AutoPath(AutoPathChoice.Red3, m_simpledrive, m_arm, m_claw));
    chooser.addOption(
        "Blue AprilTag 6", new AutoPath(AutoPathChoice.Blue6, m_simpledrive, m_arm, m_claw));
    chooser.addOption(
        "Blue AprilTag 7", new AutoPath(AutoPathChoice.Blue7, m_simpledrive, m_arm, m_claw));
    chooser.addOption(
        "Blue AprilTag 8", new AutoPath(AutoPathChoice.Blue8, m_simpledrive, m_arm, m_claw));
    // chooser.addOption("Test Path", new AutoPath(AutoPathChoice.Red1, m_simpledrive, m_arm,
    // null));

    SmartDashboard.putData("Autonomous Chooser", chooser);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_simpleDriveSubsystem::exampleCondition)
    //    .onTrue(new SimpleDriveCommand(m_simpleDriveSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.().whileTrue(m_simpleDriveSubsystem.drive(m_driverController.getX(),m_driverController.getY()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    System.out.println("Autonomous commanded");

    var autoPath = chooser.getSelected();

    if (autoPath == null) {
      return null;
    }

    // In the game we will only ever run one path per robot run but when we test we may run
    // it more then once so we need to let the drive train know if this is the first path so
    // it can reset the odometer
    boolean isFirstPath = (m_autonomousRuns++ == 0);
    PathPlannerTrajectory trajectory = autoPath.getTrajectory();
    List<EventMarker> markers = trajectory.getMarkers();
    markers.forEach((marker) -> {
      System.out.println("Marker names: " + String.join(",", marker.names));
    });
    FollowPathWithEvents command =
        new FollowPathWithEvents(
            new FollowPath(m_simpledrive, trajectory, isFirstPath),
            trajectory.getMarkers(),
            autoPath.getEventMap());

    return command;
  }
}
