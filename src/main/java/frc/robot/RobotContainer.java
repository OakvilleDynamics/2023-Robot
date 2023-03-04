// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.auto.FollowPath;
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

  private int m_autonomousRuns = 0;

  // Max velocity and max accelerations are just defaults, we should move them to constants
  PathPlannerTrajectory blue6 =
      PathPlanner.loadPath(
          "Blue April ID 6",
          new PathConstraints(
              Constants.pathDriveTrainMaxVelocity, Constants.pathDriveTrainMaxAcceleration));
  PathPlannerTrajectory blue7 =
      PathPlanner.loadPath(
          "Blue April ID 7",
          new PathConstraints(
              Constants.pathDriveTrainMaxVelocity, Constants.pathDriveTrainMaxAcceleration));
  PathPlannerTrajectory blue8 =
      PathPlanner.loadPath(
          "Blue April ID 8",
          new PathConstraints(
              Constants.pathDriveTrainMaxVelocity, Constants.pathDriveTrainMaxAcceleration));
  PathPlannerTrajectory red1 =
      PathPlanner.loadPath(
          "Red April ID 1",
          new PathConstraints(
              Constants.pathDriveTrainMaxVelocity, Constants.pathDriveTrainMaxAcceleration));
  PathPlannerTrajectory red2 =
      PathPlanner.loadPath(
          "Red April ID 2",
          new PathConstraints(
              Constants.pathDriveTrainMaxVelocity, Constants.pathDriveTrainMaxAcceleration));
  PathPlannerTrajectory red3 =
      PathPlanner.loadPath(
          "Red April ID 3",
          new PathConstraints(
              Constants.pathDriveTrainMaxVelocity, Constants.pathDriveTrainMaxAcceleration));
  PathPlannerTrajectory testPath =
      PathPlanner.loadPath(
          "TestSimplePath",
          new PathConstraints(
              Constants.pathDriveTrainMaxVelocity, Constants.pathDriveTrainMaxAcceleration));

  SendableChooser<PathPlannerTrajectory> chooser = new SendableChooser<>();

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
    chooser.addOption("Red AprilTag 1", red1);
    chooser.addOption("Red AprilTag 2", red2);
    chooser.addOption("Red AprilTag 3", red3);
    chooser.addOption("Blue AprilTag 6", blue6);
    chooser.addOption("Blue AprilTag 7", blue7);
    chooser.addOption("Blue AprilTag 8", blue8);
    chooser.addOption("Test Path", testPath);

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

    var trajectory = chooser.getSelected();

    if (trajectory == null) {
      return null;
    }

    // In the game we will only ever run one path per robot run but when we test we may run
    // it more then once so we need to let the drive train know if this is the first path so
    // it can reset the odometer
    boolean isFirstPath = (m_autonomousRuns++ == 0);
    return new FollowPath(m_simpledrive, trajectory, isFirstPath);
  }
}
