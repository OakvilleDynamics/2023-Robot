package frc.robot.components;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AutoPathChoice;
import frc.robot.Constants.ClawObjectPlacement;
import frc.robot.Constants.ClawObjectType;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.PneumaticClaw;
import frc.robot.subsystems.TalonDrive;
import java.util.HashMap;

public class AutoPath {

  private static final String PlaceObjectMarker = "PlaceObject";
  private static final String PickupObjectMarker = "PickUpObject";
  private static final String AutoLevelMarker = "ChargeStationLevel";

  PathPlannerTrajectory m_trajectory = null;
  HashMap<String, Command> m_eventMap = new HashMap<>();

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

  public AutoPath(AutoPathChoice autoPathChoice, TalonDrive drive, Arm arm, PneumaticClaw claw) {
    switch (autoPathChoice) {
      case Red1:
        // Drive forward, place cone on top poll, drive out of Community, pick up second cone,
        // park outside of community turned toward community
        m_trajectory = red1;
        m_eventMap.put(
            PlaceObjectMarker,
            new PlaceObject(arm, claw, ClawObjectType.Cone, ClawObjectPlacement.Top));
        m_eventMap.put(
            PickupObjectMarker,
            new PickupObject(arm, claw, ClawObjectType.Cone, ClawObjectPlacement.Hybrid));
        break;
      case Red2:
        // Drive forward, place cone on top poll, drive over charging station and out of Community,
        // pick up second cone, park on top of charging station level, turned toward community
        m_trajectory = red2;
        m_eventMap.put(
            PlaceObjectMarker,
            new PlaceObject(arm, claw, ClawObjectType.Cone, ClawObjectPlacement.Top));
        m_eventMap.put(
            PickupObjectMarker,
            new PickupObject(arm, claw, ClawObjectType.Cone, ClawObjectPlacement.Hybrid));
        m_eventMap.put(AutoLevelMarker, new AutoLevel(drive));
        break;
      case Red3:
        // Drive forward, place cone on top poll, drive out of Community, pick up second cone,
        // park outside of community turned toward community
        m_trajectory = red3;
        m_eventMap.put(
            PlaceObjectMarker,
            new PlaceObject(arm, claw, ClawObjectType.Cone, ClawObjectPlacement.Top));
        m_eventMap.put(
            PickupObjectMarker,
            new PickupObject(arm, claw, ClawObjectType.Cone, ClawObjectPlacement.Hybrid));
        break;
      case Blue6:
        // Drive forward, place cone on top poll, drive out of Community, pick up second cone,
        // park outside of community turned toward community
        m_trajectory = blue6;
        m_eventMap.put(
            PlaceObjectMarker,
            new PlaceObject(arm, claw, ClawObjectType.Cone, ClawObjectPlacement.Top));
        m_eventMap.put(
            PickupObjectMarker,
            new PickupObject(arm, claw, ClawObjectType.Cone, ClawObjectPlacement.Hybrid));
        break;
      case Blue7:
        // Drive forward, place cone on top poll, drive over charging station and out of Community,
        // pick up cube, park on top of charging station level, turned toward community
        m_trajectory = blue7;
        m_eventMap.put(
            PlaceObjectMarker,
            new PlaceObject(arm, claw, ClawObjectType.Cone, ClawObjectPlacement.Top));
        m_eventMap.put(
            PickupObjectMarker,
            new PickupObject(arm, claw, ClawObjectType.Cone, ClawObjectPlacement.Hybrid));
        m_eventMap.put(AutoLevelMarker, new AutoLevel(drive));
        break;
      case Blue8:
        // Drive forward, place cone on top poll, drive out of Community, pick up second cone,
        // park outside of community turned toward community
        m_trajectory = blue8;
        m_eventMap.put(
            PlaceObjectMarker,
            new PlaceObject(arm, claw, ClawObjectType.Cone, ClawObjectPlacement.Top));
        m_eventMap.put(
            PickupObjectMarker,
            new PickupObject(arm, claw, ClawObjectType.Cone, ClawObjectPlacement.Hybrid));
        break;
    }
    ;
  }

  public PathPlannerTrajectory getTrajectory() {
    return m_trajectory;
  }

  public HashMap<String, Command> getEventMap() {
    return m_eventMap;
  }
}
