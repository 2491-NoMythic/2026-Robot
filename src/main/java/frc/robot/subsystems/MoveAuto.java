// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import com.pathplanner.lib.path.*;

//import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveAuto extends Command {
  public MoveAuto(List<Waypoint> waypoints, List<RotationTarget> rotationTargets, List<PointTowardsZone> pointTowardsZones, List<ConstraintsZone> constraintZones, List<EventMarker> eventMarkers, PathConstraints globalConstraints, IdealStartingState startState, GoalEndState endState) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public PathPlannerPath createPath(List<Waypoint> waypoints, List<RotationTarget> rotationTargets, List<PointTowardsZone> pointTowardsZones, List<ConstraintsZone> constraintZones, List<EventMarker> eventMarkers, PathConstraints globalConstraints, GoalEndState endState){
     return new PathPlannerPath(waypoints, rotationTargets, pointTowardsZones, constraintZones, eventMarkers, globalConstraints, null, endState, false);
  }
  public List<Waypoint> createWaypoints(){

  }

}