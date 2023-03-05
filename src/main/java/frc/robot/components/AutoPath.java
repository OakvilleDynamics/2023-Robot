package frc.robot.components;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import frc.robot.Constants;
import frc.robot.Constants.AutoPathChoice;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.PneumaticClaw;
import frc.robot.subsystems.TalonDrive;
import java.util.List;

public class AutoPath {
  List<PathPlannerTrajectory> m_trajectory = null;

  public AutoPath(AutoPathChoice autoPathChoice, TalonDrive drive, Arm arm, PneumaticClaw claw) {
    // Determine path name
    String pathName = "";
    switch (autoPathChoice) {
      case Red1:
        // Drive forward, place cone on top poll, drive out of Community, pick up second cone,
        // park outside of community turned toward community
        pathName = "Red April ID 1";
        break;
      case Red2:
        // Drive forward, place cone on top poll, drive over charging station and out of Community,
        // pick up second cone, park on top of charging station level, turned toward community
        pathName = "Red April ID 2";
        break;
      case Red3:
        // Drive forward, place cone on top poll, drive out of Community, pick up second cone,
        // park outside of community turned toward community
        pathName = "Red April ID 3";
        break;
      case Blue6:
        // Drive forward, place cone on top poll, drive out of Community, pick up second cone,
        // park outside of community turned toward community
        pathName = "Blue April ID 6";
        break;
      case Blue7:
        // Drive forward, place cone on top poll, drive over charging station and out of Community,
        // pick up cube, park on top of charging station level, turned toward community
        pathName = "Blue April ID 7";
        break;
      case Blue8:
        // Drive forward, place cone on top poll, drive out of Community, pick up second cone,
        // park outside of community turned toward community
        pathName = "Blue April ID 8";
        break;
    }

    // This will load the file "Example Path Group.path" and generate it with a max velocity of 4
    // m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    m_trajectory =
        PathPlanner.loadPathGroup(
            pathName,
            new PathConstraints(
                Constants.pathDriveTrainMaxVelocity, Constants.pathDriveTrainMaxAcceleration));
    ;
  }

  public List<PathPlannerTrajectory> getPathGroup() {
    return m_trajectory;
  }
}
