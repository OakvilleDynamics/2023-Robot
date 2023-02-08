// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Brakes;
import frc.robot.commands.DriveTrain;
import frc.robot.subsystems.SparkMaxDrive;
import frc.robot.commands.MoveTurret;
import frc.robot.subsystems.ArmTurret;
import frc.robot.subsystems.PneumaticBrakes;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

 /* This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static final RobotContainer m_robotContainer = new RobotContainer();

  // The robot's subsystems
  public final ArmTurret m_turret = new ArmTurret();
  private final SparkMaxDrive m_simpledrive = new SparkMaxDrive();
  public final PneumaticBrakes m_Brakes = new PneumaticBrakes();

  // Autonomous Commands
  // private final Command m_placeobject = new PlaceObject(m_turret);   


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_turret.setDefaultCommand(new MoveTurret(m_turret));
    m_simpledrive.setDefaultCommand(new DriveTrain(m_simpledrive));
    m_Brakes.setDefaultCommand(new Brakes(m_Brakes));
    //m_arm.setDefaultCommand(new MoveArm(m_arm));
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
    //new Trigger(m_simpleDriveSubsystem::exampleCondition)
    //    .onTrue(new SimpleDriveCommand(m_simpleDriveSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.().whileTrue(m_simpleDriveSubsystem.drive(m_driverController.getX(),m_driverController.getY()));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }*/
}
