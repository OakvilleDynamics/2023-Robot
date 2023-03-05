package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.TalonDrive;

public class AutoLevel extends SequentialCommandGroup {

  public AutoLevel(TalonDrive drive) {
    System.out.println("Auto level Robot");

    addCommands(new PrintCommand("Auto leveling robot."));
  }
}
