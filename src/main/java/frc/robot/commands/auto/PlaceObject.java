package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClawObjectPlacement;
import frc.robot.Constants.ClawObjectType;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.PneumaticClaw;
import frc.robot.subsystems.SparkClaw;

public class PlaceObject extends SequentialCommandGroup {

  public PlaceObject(
      Arm arm,
      SparkClaw claw,
      ClawObjectType clawObjectType,
      ClawObjectPlacement clawObjectPlacement) {
    System.out.println("Placing Object");

    // Starting with baby steps, not driving in here yet, full request is to
    // Drive forward, adjust arm for proper height, drive forward, open claw, start returning
    // arm to transport location, back up, allow driver control again.

    addCommands(
        new PrintCommand("Extend arm to proper height."),
        new InstantCommand(
            () -> {
              claw.pickupObject();
            }),
        new PrintCommand("Return arm to transport location."));
  }
}
