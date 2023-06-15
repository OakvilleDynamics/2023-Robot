package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClawObjectPlacement;
import frc.robot.Constants.ClawObjectType;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.PneumaticClaw;
import frc.robot.subsystems.SparkClaw;

public class PickupObject extends SequentialCommandGroup {

  public PickupObject(
      Arm arm,
      SparkClaw claw,
      ClawObjectType clawObjectType,
      ClawObjectPlacement clawObjectPlacement) {
    System.out.println("Pickup Object");

    addCommands(
        new PrintCommand("Extend arm to proper height."),
        new InstantCommand(
            () -> {
              claw.pickupObject();
            }),
        new PrintCommand("Return arm to transport location."));
  }
}
