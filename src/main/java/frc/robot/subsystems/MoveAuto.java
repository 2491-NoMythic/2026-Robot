// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import com.pathplanner.lib.path.*;

import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveAuto extends Command {
    List<Waypoint> waypoints;
    List<RotationTarget> rotationTargets;
    List<ConstraintsZone> constraintsZones;


  public MoveAuto(List<Waypoint> waypoints, List<RotationTarget> rotationTargets, List<PointTowardsZone> pointTowardsZones, List<ConstraintsZone> constraintZones, List<EventMarker> eventMarkers, PathConstraints globalConstraints, IdealStartingState startState, GoalEndState endState) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /**
   * 
   * @param waypoints done in this format: 
   * @param rotationTargets
   * @param pointTowardsZones
   * @param constraintZones
   * @param eventMarkers
   * @param globalConstraints
   * @param endState
   * @return
   */
  public PathPlannerPath createPath(List<Waypoint> waypoints, List<RotationTarget> rotationTargets, List<ConstraintsZone> constraintZones, PathConstraints globalConstraints, GoalEndState endState){
     return new PathPlannerPath(waypoints, rotationTargets, null, constraintZones, null, globalConstraints, null, endState, false);
  }
  /**
   * @return
   */
  public List<Waypoint> createWaypoints(Waypoint ... wayPoints){
        for (Waypoint w : wayPoints){
        waypoints.add(w);
    } 
      return waypoints;
  }
  /**
   * @return
   */
  public List<RotationTarget> createRotationTargets(RotationTarget ... rotTargets){
    for (RotationTarget r : rotTargets){
        rotationTargets.add(r);
    }   
      return rotationTargets;
  }
  public List<ConstraintsZone> createConstraintsZones(ConstraintsZone ... conZones){
        for (ConstraintsZone z : conZones){
        constraintsZones.add(z);
    } 
      return constraintsZones;
  }
}