package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClawObjectPlacement;
import frc.robot.Constants.ClawObjectType;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.PneumaticClaw;

public class AutoLevel extends SequentialCommandGroup {

  public AutoLevel(
      Arm arm,
      PneumaticClaw claw,
      ClawObjectType clawObjectType,
      ClawObjectPlacement clawObjectPlacement) {
    System.out.println("Auto level Robot");

    addCommands(
        new InstantCommand(
            () -> {
              System.out.println("Auto leveling robot.");
            }));
  }
}
